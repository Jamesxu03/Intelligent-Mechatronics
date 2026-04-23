# Self-Balancing Segway - PyBench STM32

MicroPython software for a self-balancing Segway on a custom PyBench (STM32) board.

## Hardware (per Appendix A)

- **DIP:** SW2=X6, SW1=Y3, SW0=Y8
- **Pot:** X11 (ADC 0–4095)
- **Motor A:** PWM X1, AIN1 X3, AIN2 X4 (TB6612)
- **Motor B:** PWM X2, BIN1 X7, BIN2 X8
- **MPU6050:** I2C1 (X9/X10), addr 0x68
- **OLED_938:** I2C2 (Y9/Y10), addr 0x3D, RES Y8

## File Structure

| File | Purpose |
|------|---------|
| `main.py` | Entry point; DIP switch menu; runs `challenge_5` when switch = '010' |
| `challenge_5.py` | Self-balancing routine: standby, live PID tuning, 200 Hz control loop |
| `pid.py` | PID controller class; output clamped to ±100 |
| `config.py` | Pin mappings, calibration offsets, filter parameters |
| `motor_driver.py` | Motor control: `A_stop()`, `B_stop()`, `drive(pwm_a, pwm_b)` |
| `mpu6050_driver.py` | MPU6050 accel/gyro reads (g, rad/s) |
| `oled_display.py` | Wrapper for OLED_938 |
| `oled_938.py` | OLED_938 driver (128x64, I2C2, RES Y8) |
| `i2c_bus.py` | I2C bus 1 (MPU) and bus 2 (OLED) |

## Calibration

1. **config.py** – `COMPLEMENTARY_ALPHA` (0.93–0.97)
2. **challenge_5.py** – `ACCEL_PITCH_SIGN`, `GYRO_PITCH_SIGN` (default +1)
3. **MPU6050** – Gyro zero bias auto-calibrated at init (100 samples, robot stationary)  

## Units

- **Pitch:** radians (interior: degrees are converted where needed)
- **Gyro:** rad/s
- **Time:** `dt` in seconds (5 ms = 0.005 s)
- **PWM:** -100 to +100

## Usage

1. Set DIP switch to `010` (middle switch on, others off).
2. Power on. `main.py` runs automatically.
3. OLED shows “Segway Ready – Press USER to begin”.
4. Press USER → Live PID tuning. Turn pot for Kp/Ki/Kd; press USER to advance.
5. After Kd → Balancing starts at 200 Hz.
