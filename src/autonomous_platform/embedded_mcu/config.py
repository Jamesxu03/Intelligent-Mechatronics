"""
config.py - Hardware configuration and calibration for PyBench Self-Balancing Segway

Hardware assignments per Appendix A pinout.
"""

# =============================================================================
# DIP Switch Configuration (3-bit)
# SW2=MSB, SW1, SW0=LSB -> '010' = challenge_5
# NOTE: SW0 was remapped from Y8 to X8 to avoid conflict with OLED RES pin.
# =============================================================================
DIP_PINS = ['X6', 'Y3', 'X8']   # Fixed conflict with OLED RES (was Y8)

# =============================================================================
# USER Button
# =============================================================================
USE_BUILTIN_SWITCH = True
USER_BUTTON_PIN = 'X17'   # Per Appendix A; fallback when USE_BUILTIN_SWITCH=False

# =============================================================================
# Potentiometer (5k ohm)
# ADC 12-bit (0-4095)
# =============================================================================
POT_PIN = 'X11'

# =============================================================================
# Motor Driver (TB6612)
# Motor A: PWM X1, AIN1 X3, AIN2 X4
# Motor B: PWM X2, BIN1 X7, BIN2 X8
# =============================================================================
MOTOR_A_PWM_PIN = 'X1'
MOTOR_A_AIN1_PIN = 'X3'
MOTOR_A_AIN2_PIN = 'X4'
MOTOR_B_PWM_PIN = 'X2'
MOTOR_B_BIN1_PIN = 'X7'
MOTOR_B_BIN2_PIN = 'X8'

# =============================================================================
# MPU6050 IMU - I2C Bus 1 (X9 SCL, X10 SDA)
# =============================================================================
MPU6050_I2C_BUS = 1
MPU6050_I2C_SCL = 'X9'
MPU6050_I2C_SDA = 'X10'
MPU6050_ADDR = 0x68

# Raw register scaling: 131 LSB/(deg/s), 16384 LSB/g
GYRO_SCALE = 131.0
ACCEL_SCALE = 16384.0

# =============================================================================
# OLED_938 - I2C Bus 2 (Y9 SCL, Y10 SDA), addr 0x3D, RES Y8
# =============================================================================
OLED_I2C_BUS = 2
OLED_I2C_ADDR = 0x3D
OLED_RES_PIN = 'Y8'

# =============================================================================
# Complementary Filter (α: 0.93–0.97)
# =============================================================================
COMPLEMENTARY_ALPHA = 0.96

# =============================================================================
# Control Loop: 200 Hz = 5000 µs
# =============================================================================
LOOP_PERIOD_US = 5000
LOOP_FREQ_HZ = 200
