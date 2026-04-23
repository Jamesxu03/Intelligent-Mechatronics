"""
trajectory_tracking.py - Bridges visual perception offsets to PID loops.
Runs directly on the Segway's MicroPython controller receiving Bluetooth metrics.
"""

from pid import PIDController

class AutonomousTrajectoryTracker:
    def __init__(self):
        # We initialize a unique PID controller strictly to balance steering orientation 
        # (Yaw/Drive instead of default Pitch) 
        self.steering_pid = PIDController(kp=1.5, ki=0.1, kd=0.5)
        
    def calculate_steering_pwm(self, center_offset_px, current_yaw_rate, dt=0.1):
        """
        Takes raw continuous tracking outputs from perception.py, computes expected
        drive correction values and feeds them back into the dual-motor subsystem.
        """
        # Our setpoint is 0px error (meaning we are purely aligned to the center line bounds)
        setpoint = 0.0
        
        # Calculate the PWM bias for left and right wheels
        steering_bias = self.steering_pid.update(
            pitch=center_offset_px,     # Reusing the existing PID object (treating offset as error)
            pitch_rate=current_yaw_rate, 
            setpoint=setpoint, 
            dt_sec=dt
        )
        
        return steering_bias

def run_autonomous_mode():
    """
    Challenge 6: Stand-alone hook into the Bluetooth telemetry module.
    """
    print("[SYSTEM] Engaging Autonomous Perception Driven Trajectory PID...")
    tracker = AutonomousTrajectoryTracker()
    
    print("[SYSTEM] Embedded stub. Execution routes to MicroPython MCU telemetry across UART/BLE via host platform simulation architecture.")
