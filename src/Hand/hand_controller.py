"""
Main entry point for redundant gesture processing.
Validates physical flexible sensor data against MediaPipe visual outputs before confirming a robot sequence.
"""

import cv2
import mediapipe as mp
import serial
import threading
import time
import logging
from enum import Enum

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

class Gesture(Enum):
    ROCK = 0
    PAPER = 1
    SCISSORS = 2
    UNKNOWN = -1

class VisualGestureRecognizer:
    """Uses Google MediaPipe Vision API for robust low-latency gesture detection."""
    def __init__(self, confidence_threshold=0.75):
        try:
            import mediapipe as mp
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=confidence_threshold,
                min_tracking_confidence=confidence_threshold
            )
            self.draw = mp.solutions.drawing_utils
        except Exception as e:
            logging.error(f"MediaPipe Vision API unavailable on this architecture: {e}")
            self.hands = None
        self.tip_ids = [4, 8, 12, 16, 20] # Thumb, Index, Middle, Ring, Pinky
        
    def extract_gesture(self, frame) -> Gesture:
        """Processes RGB video frame and evaluates 3D hand landmarks."""
        if self.hands is None:
            return Gesture.UNKNOWN
            
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb_frame)
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                fingers = []
                
                # Check Thumb (pseudo-heuristics based on relative X/Y depending on handedness)
                if hand_landmarks.landmark[self.tip_ids[0]].x < hand_landmarks.landmark[self.tip_ids[0] - 1].x:
                    fingers.append(1)
                else:
                    fingers.append(0)
                    
                # Check 4 Fingers
                for id in range(1, 5):
                    if hand_landmarks.landmark[self.tip_ids[id]].y < hand_landmarks.landmark[self.tip_ids[id] - 2].y:
                        fingers.append(1)
                    else:
                        fingers.append(0)
                
                open_count = fingers.count(1)
                
                if open_count == 0:
                    return Gesture.ROCK
                elif open_count == 5 or open_count == 4:
                    return Gesture.PAPER
                elif fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 0:
                    return Gesture.SCISSORS
                    
        return Gesture.UNKNOWN

class PhysicalSensorInterface:
    """Manages high-frequency serial streaming logic from the custom PCB/Arduino framework."""
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.connected = True
            logging.info(f"Connected to motor control and sensor PCB on {port}.")
        except serial.SerialException:
            self.connected = False
            logging.warning("Failed to connect to PCB; simulating hardware flow for debug.")
            
        self.latest_physical_move = Gesture.UNKNOWN
        self.lock = threading.Lock()
        
    def read_telemetry_loop(self):
        while True:
            if self.connected and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "PHYSICAL_MOVE:" in line:
                    move_code = int(line.split(":")[1])
                    with self.lock:
                        self.latest_physical_move = Gesture(move_code)
            time.sleep(0.01)
            
    def get_latest_move(self):
        with self.lock:
            return self.latest_physical_move

class SystemIntegrator:
    """Core synchronization protocol to enforce redundancy."""
    def __init__(self):
        self.vision = VisualGestureRecognizer()
        self.hardware = PhysicalSensorInterface(port='/dev/cu.usbserial-0001') # Standard MAC layout
        self.telemetry_thread = threading.Thread(target=self.hardware.read_telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        
    def run_fusion_pipeline(self):
        try:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                logging.error("CRITICAL EXCEPTION: Main RGB Camera Feed Offline. Halting.")
                return
            logging.info("Starting Dual-Redundant Fusion Pipeline...")
        except Exception as e:
            logging.error(f"FATAL BOOT EXCEPTION: {e}")
            return
        
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                break
                
            # 1. Obtain visual trajectory estimate
            visual_prediction = self.vision.extract_gesture(frame)
            
            # 2. Obtain physical system telemetry bounds
            physical_prediction = self.hardware.get_latest_move()
            
            # 3. Synchronized validation checking
            if visual_prediction != Gesture.UNKNOWN and physical_prediction != Gesture.UNKNOWN:
                if visual_prediction == physical_prediction:
                    logging.info(f"VERIFIED SEQUENCE: Both domains confirm -> {visual_prediction.name}")
                    # Issue command to backend/robotic firmware
                else:
                    logging.warning(f"CONFLICT DETECTED: Visual ({visual_prediction.name}) != Physical ({physical_prediction.name})")

            # Output overlay logic for debug
            cv2.putText(frame, f"Vision: {visual_prediction.name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f"Sensors: {physical_prediction.name}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            cv2.imshow("Redundancy Pipeline", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    system = SystemIntegrator()
    system.run_fusion_pipeline()
