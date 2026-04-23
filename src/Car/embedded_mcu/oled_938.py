"""
oled_938.py - OLED_938 display driver (128x64, I2C bus 2, addr 0x3D, RES Y8)
"""

import pyb
import framebuf
from config import OLED_I2C_ADDR, OLED_RES_PIN

WIDTH = 128
HEIGHT = 64
PAGES = 8


class OLED_938:
    """
    OLED_938: 128x64 I2C display with hardware RES.
    I2C bus 2, addr 0x3D. RES pin on Y8.
    """

    def __init__(self):
        from i2c_bus import get_i2c
        self._i2c = get_i2c(2)
        self._addr = OLED_I2C_ADDR
        self._res = pyb.Pin(OLED_RES_PIN, pyb.Pin.OUT_PP)
        self._buf = bytearray(WIDTH * PAGES)
        self._fb = framebuf.FrameBuffer(self._buf, WIDTH, HEIGHT, framebuf.MONO_VLSB)
        self._hw_reset()
        self._init_display()

    def _hw_reset(self):
        """RES active low: pull low, delay, pull high, delay."""
        self._res.low()
        pyb.delay(10)
        self._res.high()
        pyb.delay(10)

    def _write_cmd(self, cmd):
        self._i2c.send(bytes([0x00, cmd]), self._addr)

    def _init_display(self):
        """SSD1306/SH1106 init sequence."""
        for c in (0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
                  0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
                  0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF):
            self._write_cmd(c)
        self.fill(0)
        self.show()

    def fill(self, c):
        self._fb.fill(c)

    def text(self, s, x, y, c=1):
        self._fb.text(str(s), x, y, c)

    def clear(self):
        """Clear display buffer (compatible with official oled_938 API)."""
        self._fb.fill(0)

    def draw_text(self, x, y, s):
        """Draw text at (x,y) (compatible with official oled_938 API)."""
        self._fb.text(str(s), x, y, 1)

    def display(self):
        """Push buffer to screen (compatible with official oled_938 API)."""
        self.show()

    def show(self):
        self._write_cmd(0x21)
        self._write_cmd(0)
        self._write_cmd(WIDTH - 1)
        self._write_cmd(0x22)
        self._write_cmd(0)
        self._write_cmd(PAGES - 1)
        self._i2c.send(bytes([0x40]) + self._buf, self._addr)
