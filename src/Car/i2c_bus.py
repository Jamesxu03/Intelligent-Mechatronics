"""
i2c_bus.py - I2C bus access for MPU6050 (bus 1) and OLED (bus 2)
"""

import pyb

_i2c1 = None
_i2c2 = None


def get_i2c(bus_num=1):
    """Return I2C instance. bus_num 1=MPU6050, 2=OLED."""
    global _i2c1, _i2c2
    if bus_num == 1:
        if _i2c1 is None:
            _i2c1 = pyb.I2C(1, pyb.I2C.MASTER, baudrate=400000)
        return _i2c1
    elif bus_num == 2:
        if _i2c2 is None:
            _i2c2 = pyb.I2C(2, pyb.I2C.MASTER, baudrate=400000)
        return _i2c2
    raise ValueError("bus_num must be 1 or 2")
