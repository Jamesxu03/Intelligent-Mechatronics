"""
challenge_5.py - Self-Balancing Segway Main Routine

Architecture:
  1. Initialization & Standby - prompt USER button to begin
  2. Live PID Tuning - pot selects Kp/Ki/Kd, USER advances
  3. Control Loop @ 200Hz - complementary filter + PID + motor drive
  4. Safety - try/finally ensures motors stop on any exit/exception
"""

import pyb
import math
from config import (
    USE_BUILTIN_SWITCH, USER_BUTTON_PIN,
    POT_PIN, COMPLEMENTARY_ALPHA, LOOP_PERIOD_US,
)
from pid import PIDController
from motor_driver import motor
from mpu6050_driver import MPU6050
from oled_display import OLED


# =============================================================================
# Calibration: Adjust for your IMU mounting orientation
# Pitch from accel: atan2(ax, az) when X=forward, Z=up. Reverse signs if needed.
# Gyro pitch rate: typically Y axis. Use -gy if direction is inverted.
# =============================================================================
ACCEL_PITCH_SIGN = 1    # +1 or -1 depending on orientation
GYRO_PITCH_SIGN = 1     # +1 or -1 for pitch-rate axis


def get_user_button():
    """Return True if USER button is pressed."""
    if USE_BUILTIN_SWITCH:
        return pyb.Switch().value()
    return pyb.Pin(USER_BUTTON_PIN, pyb.Pin.IN, pyb.Pin.PULL_UP).value() == 0


def wait_for_user_release():
    """Debounce: wait until USER button is released."""
    while get_user_button():
        pyb.delay(20)


def accel_to_pitch_rad(ax, ay, az):
    """
    Compute pitch angle (radians) from accelerometer.
    Assumes: X forward, Y left, Z up. Pitch = rotation about Y.
    ρ = atan2(ax, az) when upright. Adjust ACCEL_PITCH_SIGN if inverted.
    """
    # Avoid div-by-zero; use atan2 for full quadrant
    pitch_accel = math.atan2(ACCEL_PITCH_SIGN * ax, az)
    return pitch_accel


def run_challenge_5():
    """Main self-balancing routine."""
    oled = OLED()
    imu = MPU6050()
    pot = pyb.ADC(pyb.Pin(POT_PIN))
    
    # -------- Phase 1: Initialization & Standby --------
    motor.A_stop()
    motor.B_stop()
    
    oled.clear()
    oled.draw_text(0, 0, "Segway Ready")
    oled.draw_text(0, 16, "Press USER to begin")
    oled.display()
    
    while not get_user_button():
        pyb.delay(50)
    wait_for_user_release()
    
    # -------- Phase 2: Live PID Tuning --------
    # Pot range 0-4095 maps to Kp/Ki/Kd. Adjust max gains for your robot.
    KP_MAX, KI_MAX, KD_MAX = 50.0, 5.0, 50.0
    param_index = 0  # 0=Kp, 1=Ki, 2=Kd
    
    kp = 10.0
    ki = 0.5
    kd = 5.0
    
    pid = PIDController(kp, ki, kd)
    
    while param_index < 3:
        raw = pot.read()
        val = raw / 4095.0  # 0.0 to 1.0
        
        if param_index == 0:
            kp = val * KP_MAX
            pid.set_gains(kp, ki, kd)
            oled.clear()
            oled.draw_text(0, 0, "Kp: {:.1f}".format(kp))
            oled.draw_text(0, 16, "Pot->Kp. USER=next")
            oled.draw_text(0, 32, "Kp={:.2f} Ki={:.2f}".format(kp, ki))
            oled.draw_text(0, 48, "Kd={:.2f}".format(kd))
        elif param_index == 1:
            ki = val * KI_MAX
            pid.set_gains(kp, ki, kd)
            oled.clear()
            oled.draw_text(0, 0, "Ki: {:.2f}".format(ki))
            oled.draw_text(0, 16, "Pot->Ki. USER=next")
            oled.draw_text(0, 32, "Kp={:.2f} Ki={:.2f}".format(kp, ki))
            oled.draw_text(0, 48, "Kd={:.2f}".format(kd))
        else:
            kd = val * KD_MAX
            pid.set_gains(kp, ki, kd)
            oled.clear()
            oled.draw_text(0, 0, "Kd: {:.1f}".format(kd))
            oled.draw_text(0, 16, "Pot->Kd. USER=start")
            oled.draw_text(0, 32, "Kp={:.2f} Ki={:.2f}".format(kp, ki))
            oled.draw_text(0, 48, "Kd={:.2f}".format(kd))
        
        oled.display()
        
        if get_user_button():
            wait_for_user_release()
            param_index += 1
        
        pyb.delay(50)
    
    # Brief ready message
    oled.clear()
    oled.draw_text(0, 24, "Balancing...")
    oled.display()
    pyb.delay(500)
    
    # -------- Phase 3: Control Loop @ 200Hz --------
    alpha = COMPLEMENTARY_ALPHA
    dt_sec = LOOP_PERIOD_US / 1_000_000.0  # 5ms = 0.005 seconds
    setpoint = 0.0  # Target pitch (radians) = upright
    integral_err = 0.0
    integral_max = 10.0  # Anti-windup
    theta = 0.0  # Pitch estimate (radians)
    
    # Initial accel read for theta
    ax, ay, az, gx, gy, gz = imu.read_accel_gyro()
    theta = accel_to_pitch_rad(ax, ay, az)
    
    loop_start = pyb.micros()
    
    try:
        while True:
            # Wait until 5000 microseconds have elapsed (200Hz)
            while pyb.elapsed_micros(loop_start) < LOOP_PERIOD_US:
                pass
            loop_start = pyb.micros()
            
            # Read IMU (accel in g, gyro in rad/s)
            ax, ay, az, gx, gy, gz = imu.read_accel_gyro()
            
            # Pitch rate: gyro Y axis = rotation about lateral axis (rad/s)
            pitch_rate = GYRO_PITCH_SIGN * gy
            
            # Complementary filter: θ_new = α*(θ_old + θ̇*dt) + (1-α)*ρ
            # θ_old, θ̇ in rad/s, dt in seconds, ρ from accel (radians)
            rho = accel_to_pitch_rad(ax, ay, az)
            theta = alpha * (theta + pitch_rate * dt_sec) + (1.0 - alpha) * rho
            
            # Integral with anti-windup
            e = theta - setpoint
            integral_err += e * dt_sec
            integral_err = max(-integral_max, min(integral_max, integral_err))
            
            # PID output: PWM in [-100, 100]
            pwm = pid.update(theta, pitch_rate, setpoint, integral_err)
            
            # Drive both motors (differential drive uses same value for forward/back)
            motor.drive(pwm, pwm)
    
    finally:
        # -------- Safety Cutoff --------
        # Motors MUST stop on any exit, crash, or exception
        motor.A_stop()
        motor.B_stop()
