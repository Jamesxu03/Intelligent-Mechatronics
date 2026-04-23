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
        self.integral_error = 0.0
        
    def calculate_steering_pwm(self, center_offset_px, current_yaw_rate):
        """
        Takes raw continuous tracking outputs from perception.py, computes expected
        drive correction values and feeds them back into the dual-motor subsystem.
        """
        # Our setpoint is 0px error (meaning we are purely aligned to the center line bounds)
        setpoint = 0.0
        
        self.integral_error += center_offset_px
        # Clamp integral windup
        self.integral_error = max(-100, min(100, self.integral_error))
        
        # Calculate the PWM bias for left and right wheels
        steering_bias = self.steering_pid.update(
            pitch=center_offset_px,     # Reusing the existing PID object (treating offset as error)
            pitch_rate=current_yaw_rate, 
            setpoint=setpoint, 
            integral_err=self.integral_error
        )
        
        return steering_bias

def run_autonomous_mode():
    """
    Challenge 6: Stand-alone hook into the Bluetooth telemetry module.
    """
    print("[SYSTEM] Engaging Autonomous Perception Driven Trajectory PID...")
    tracker = AutonomousTrajectoryTracker()
    
    # Loop over mock bluetooth serial reads (Simulated hardware constraints)
    while True:
        # 1. Fetch visual offset via UART stream from Host PC
        # offset = uart.read()
        latest_cv_offset = 12.5 # Mock data 12px off-center
        mock_yaw_rate = 0.1
        
        # 2. Compute PID constraint 
        correction_pwm = tracker.calculate_steering_pwm(latest_cv_offset, mock_yaw_rate)
        
        # 3. Alter Base Balance routine
        # motor.drive(base_pwm + correction_pwm, base_pwm - correction_pwm)
        
        # Simulation delay
        # pyb.delay(10)
        break # Break directly in simulation
        
    print("[SYSTEM] Routine Terminated.")
