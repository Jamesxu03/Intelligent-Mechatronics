import cv2
import numpy as np
import logging
from typing import List, Tuple

logging.basicConfig(level=logging.INFO)

class AutonomousPerceptionModule:
    """
    Continuous software-based perception logic executing lane detection and
    obstacle identification utilizing Canny Edge detection, Hough Transforms, 
    and contour masking.
    """
    
    def __init__(self, camera_index=0):
        try:
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened() and camera_index != "dummy":
                logging.error(f"CRITICAL: Hardware fault. Failed to initialize camera index {camera_index}")
                raise ConnectionError("Camera offline.")
            
            # Use default parameters if dummy
            if camera_index == "dummy":
                self.frame_width = 640
                self.frame_height = 480
            else:
                self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                
            logging.info(f"Initialized autonomous perception with resolution {self.frame_width}x{self.frame_height}")
        except Exception as e:
            logging.error(f"FATAL EXCEPTION in AutonomousPerceptionModule: {e}")

    def detect_lane_offset(self, frame: np.ndarray) -> int:
        """
        Calculates the central trajectory offset necessary for PID tracking.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # Region of Interest: Lower half of the camera frame
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height * 0.4)),
            (0, int(height * 0.4)),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Probabilistic Hough Line Transform
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, maxLineGap=50)
        
        left_lines = []
        right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if slope < -0.5: # Left lane
                    left_lines.append((x1, y1, x2, y2))
                elif slope > 0.5: # Right lane
                    right_lines.append((x1, y1, x2, y2))
                    
        # Approximate offset from center using average line bounds
        center_x = width // 2
        computed_lane_center = center_x
        
        if left_lines and right_lines:
            left_x_avg = int(np.mean([x1 for l in left_lines for x1 in (l[0], l[2])]))
            right_x_avg = int(np.mean([x1 for l in right_lines for x1 in (l[0], l[2])]))
            computed_lane_center = (left_x_avg + right_x_avg) // 2

        offset_error = computed_lane_center - center_x
        return offset_error

    def identify_obstacles(self, frame: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Obstacle avoidance routine utilizing color thresholding and bounding box extraction.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define arbitrary threshold for obstacles (e.g. standard red traffic cones)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # Minimum area to be considered an obstacle
                x, y, w, h = cv2.boundingRect(cnt)
                obstacles.append((x, y, w, h))
                
        return obstacles

    def process_pipeline(self):
        """Host-to-vehicle bridge simulation."""
        logging.info("Beginning Host PC -> Vehicle Telemetry stream...")
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
                
            offset = self.detect_lane_offset(frame)
            obstacles = self.identify_obstacles(frame)
            
            # Simulated telemetry transmission to the PyBoard main controller
            if len(obstacles) > 0:
                logging.warning(f"OBSTACLE DETECTED at {obstacles[0]}: Issuing emergency PID halt.")
            else:
                logging.info(f"PID Center Offset Error: {offset}px -> Transmitting to Trajectory Tracker.")
            
            cv2.imshow("Vehicle Autonomous Perception Dashboard", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    cv_module = AutonomousPerceptionModule()
    cv_module.process_pipeline()
