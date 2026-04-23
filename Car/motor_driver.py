"""
motor_driver.py - TB6612 motor driver for Self-Balancing Segway

Timer 2 @ 1000 Hz. Motor A: Ch1, Motor B: Ch2.
TB6612: Forward = IN1=L, IN2=H; Stop = IN1=L, IN2=L; Reverse = IN1=H, IN2=L.
"""

import pyb
from config import (
    MOTOR_A_PWM_PIN, MOTOR_A_AIN1_PIN, MOTOR_A_AIN2_PIN,
    MOTOR_B_PWM_PIN, MOTOR_B_BIN1_PIN, MOTOR_B_BIN2_PIN,
)


class MotorDriver:
    """
    Dual DC motor driver for TB6612.
    PWM: Timer 2 @ 1000 Hz, Ch1 (A), Ch2 (B).
    """

    def __init__(self):
        self._tim = pyb.Timer(2, freq=1000)
        self._pwm_a = self._tim.channel(1, pyb.Timer.PWM, pin=pyb.Pin(MOTOR_A_PWM_PIN))
        self._pwm_b = self._tim.channel(2, pyb.Timer.PWM, pin=pyb.Pin(MOTOR_B_PWM_PIN))
        self._ain1 = pyb.Pin(MOTOR_A_AIN1_PIN, pyb.Pin.OUT_PP)
        self._ain2 = pyb.Pin(MOTOR_A_AIN2_PIN, pyb.Pin.OUT_PP)
        self._bin1 = pyb.Pin(MOTOR_B_BIN1_PIN, pyb.Pin.OUT_PP)
        self._bin2 = pyb.Pin(MOTOR_B_BIN2_PIN, pyb.Pin.OUT_PP)

    def _set_motor(self, pwm_ch, in1, in2, pwm_val):
        """
        TB6612: Forward=IN1=L,IN2=H; Reverse=IN1=H,IN2=L; Stop=IN1=L,IN2=L.
        pwm_val: -100 to +100.
        """
        pwm_val = max(-100, min(100, pwm_val))
        if pwm_val == 0:
            in1.low()
            in2.low()
        elif pwm_val > 0:
            in1.low()
            in2.high()
        else:
            in1.high()
            in2.low()
            pwm_val = -pwm_val
        pwm_ch.pulse_width_percent(int(pwm_val))

    def A_stop(self):
        """Stop motor A (IN1=L, IN2=L)."""
        self._ain1.low()
        self._ain2.low()
        self._pwm_a.pulse_width_percent(0)

    def B_stop(self):
        """Stop motor B (IN1=L, IN2=L)."""
        self._bin1.low()
        self._bin2.low()
        self._pwm_b.pulse_width_percent(0)

    def drive(self, pwm_a, pwm_b):
        """Drive both motors. pwm_a, pwm_b: -100 to +100."""
        self._set_motor(self._pwm_a, self._ain1, self._ain2, pwm_a)
        self._set_motor(self._pwm_b, self._bin1, self._bin2, pwm_b)


motor = MotorDriver()
