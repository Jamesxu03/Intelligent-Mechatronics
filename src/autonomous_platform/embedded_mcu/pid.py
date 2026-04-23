"""
pid.py - PID Controller for Self-Balancing Segway

Control equation: w(t) = Kp*e(t) + Kd*ė(t) + Ki*∫e(τ)dτ
Units: pitch in radians, rate in rad/s, output clamped to ±100 (PWM drive).
"""


class PIDController:
    """
    Proportional-Integral-Derivative controller for pitch stabilization.
    
    Inputs:
        pitch        : Current pitch angle (radians, positive = tilt forward)
        pitch_rate   : Rate of change of pitch (rad/s, from gyro)
        setpoint     : Target pitch angle (radians, typically 0 for upright)
        integral_err : Cumulative integral of error (caller maintains for anti-windup)
    
    Output:
        PWM drive value in range [-100, +100]
    """
    
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        """
        Initialize PID gains.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._pwm_limit = 100  # Strict limit: output clamped to ±100
        self._integral = 0.0
    
    def update(self, pitch, pitch_rate, setpoint, dt_sec=0.005):
        """
        Compute PID output.
        
        Args:
            pitch        : Current pitch angle (radians)
            pitch_rate   : Rate of change of pitch (rad/s)
            setpoint     : Target pitch (radians)
            integral_err : Cumulative error integral (rad*s)
        
        Returns:
            pwm: PWM drive value in [-100, 100]
                 Positive = drive forward, negative = drive backward
        """
        # Error: positive when tilted forward (pitch > setpoint)
        e = pitch - setpoint
        
        # Derivative term: ė = d(pitch - setpoint)/dt = pitch_rate (setpoint is constant)
        e_dot = pitch_rate
        
        # PID equation: w(t) = Kp*e(t) + Kd*ė(t) + Ki*∫e(τ)dτ
        p_term = self.kp * e
        d_term = self.kd * e_dot
        self._integral += e * dt_sec
        integral_max = 10.0
        self._integral = max(-integral_max, min(integral_max, self._integral))
        i_term = self.ki * self._integral
        
        pwm = p_term + d_term + i_term
        
        # Strict clamp to ±100
        if pwm > self._pwm_limit:
            pwm = self._pwm_limit
        elif pwm < -self._pwm_limit:
            pwm = -self._pwm_limit
        
        return pwm
    
    def set_gains(self, kp, ki, kd):
        """Update PID gains (e.g. from potentiometer during live tuning)."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
