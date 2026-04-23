"""
oled_display.py - Wrapper for OLED_938

Uses oled_938.py driver (128x64, I2C bus 2, addr 0x3D, RES Y8).
"""

from oled_938 import OLED_938


class OLED(OLED_938):
    """Compatibility alias for OLED_938."""
    pass
