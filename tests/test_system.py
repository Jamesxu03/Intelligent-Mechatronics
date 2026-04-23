import sys
import os
import numpy as np

# Adjust the path to import src modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from Hand.hand_controller import VisualGestureRecognizer, Gesture
from Car.perception import AutonomousPerceptionModule

def test_visual_gesture_empty_frame():
    """TDD Edge Case: Ensure the gesture recognizer doesn't crash on an empty frame."""
    recognizer = VisualGestureRecognizer()
    blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    try:
        result = recognizer.extract_gesture(blank_frame)
    except Exception as e:
        assert False, f"extract_gesture raised an exception: {e}"
        
    assert result == Gesture.UNKNOWN, "Empty frame should return UNKNOWN gesture"

def test_car_obstacle_detection_empty():
    """TDD Edge Case: Ensure standard HSV ranges do not falsely flag a blank camera feed."""
    # We pass a dummy path to avoid connecting to the physical webcam during CI/CD tests
    mod = AutonomousPerceptionModule(camera_index="dummy_string_prevents_init") 
    blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    try:
        obstacles = mod.identify_obstacles(blank_frame)
    except Exception as e:
        assert False, f"identify_obstacles raised an exception: {e}"
        
    assert len(obstacles) == 0, "No obstacles should be found in a completely black frame"

def test_car_lane_offset_validation():
    """TDD Calculation: Check if a blank image results in a centered 0 offset naturally."""
    mod = AutonomousPerceptionModule(camera_index="dummy") 
    blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    offset = mod.detect_lane_offset(blank_frame)
    
    assert offset == 0, "A completely symmetrical blank frame should yield a perfect 0 center offset error."
