"""
mpu6050_driver.py - MPU6050 IMU driver for pitch estimation

Raw scaling: GYRO_SCALE=131 LSB/(deg/s), ACCEL_SCALE=16384 LSB/g.
Output: accel in g, gyro in rad/s. Dynamic gyro offset calibration at init.
"""

import pyb
import math
from config import (
    MPU6050_ADDR, GYRO_SCALE, ACCEL_SCALE,
)

# MPU6050 register addresses
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B

# Scaling: 131 LSB/(deg/s) -> rad/s
DEG_TO_RAD = math.pi / 180.0
GYRO_LSB_TO_RAD_S = (1.0 / GYRO_SCALE) * DEG_TO_RAD
ACCEL_LSB_TO_G = 1.0 / ACCEL_SCALE


class MPU6050:
    """
    MPU6050 driver. Calibrates gyro zero bias at init (100 samples when stationary).
    """

    def __init__(self, calibrate_gyro=True, n_samples=100):
        from i2c_bus import get_i2c
        self._i2c = get_i2c(1)
        self._addr = MPU6050_ADDR
        self._gyro_offsets = (0.0, 0.0, 0.0)
        self._wake()
        pyb.delay(100)
        if calibrate_gyro:
            self._calibrate_gyro(n_samples)
    
    def _wake(self):
        """Wake MPU6050 from sleep (required after power-up)."""
        self._i2c.mem_write(0x00, self._addr, REG_PWR_MGMT_1)
    
    def _read_raw(self, reg, nbytes):
        """Read nbytes from register, return bytes."""
        return self._i2c.mem_read(nbytes, self._addr, reg)
    
    def _bytes_to_s16(self, high, low):
        val = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def _calibrate_gyro(self, n_samples):
        """Average gyro over n_samples while robot is stationary."""
        gx_sum = gy_sum = gz_sum = 0.0
        for _ in range(n_samples):
            buf = self._read_raw(REG_ACCEL_XOUT_H, 14)
            gx_sum += self._bytes_to_s16(buf[8], buf[9])
            gy_sum += self._bytes_to_s16(buf[10], buf[11])
            gz_sum += self._bytes_to_s16(buf[12], buf[13])
            pyb.delay(2)
        n = float(n_samples)
        self._gyro_offsets = (
            (gx_sum / n) * GYRO_LSB_TO_RAD_S,
            (gy_sum / n) * GYRO_LSB_TO_RAD_S,
            (gz_sum / n) * GYRO_LSB_TO_RAD_S,
        )

    def read_accel_gyro(self):
        """
        Read accelerometer (g) and gyroscope (rad/s).
        
        Returns:
            (ax, ay, az, gx, gy, gz)
            - accel in g (float)
            - gyro in rad/s (float)
        
        Axes: chip axes. For pitch (forward tilt):
          - Accel pitch from X,Z: atan2(ax, az)
          - Gyro pitch rate from Y axis (rotation about Y)
        
        CALIBRATION: Apply GYRO_*_OFFSET in config if you observe drift at rest.
        """
        # Read 14 bytes: accel (6) + temp (2) + gyro (6)
        buf = self._read_raw(REG_ACCEL_XOUT_H, 14)
        ax_raw = self._bytes_to_s16(buf[0], buf[1])
        ay_raw = self._bytes_to_s16(buf[2], buf[3])
        az_raw = self._bytes_to_s16(buf[4], buf[5])
        gx_raw = self._bytes_to_s16(buf[8], buf[9])
        gy_raw = self._bytes_to_s16(buf[10], buf[11])
        gz_raw = self._bytes_to_s16(buf[12], buf[13])
        
        ax = ax_raw * ACCEL_LSB_TO_G
        ay = ay_raw * ACCEL_LSB_TO_G
        az = az_raw * ACCEL_LSB_TO_G
        
        gx = gx_raw * GYRO_LSB_TO_RAD_S - self._gyro_offsets[0]
        gy = gy_raw * GYRO_LSB_TO_RAD_S - self._gyro_offsets[1]
        gz = gz_raw * GYRO_LSB_TO_RAD_S - self._gyro_offsets[2]
        return (ax, ay, az, gx, gy, gz)
