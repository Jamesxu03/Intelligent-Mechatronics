"""
main.py - Entry point for PyBench Self-Balancing Segway

Menu system driven by 3-bit DIP switch:
  - '010' (binary 2) -> challenge_5: Self-Balancing Segway routine
  - '100' (binary 4) -> trajectory_tracking: Autonomous CV PID Routine
"""

import pyb
from config import DIP_PINS


def read_dip_switch():
    """
    Read 3-bit DIP switch.
    DIP_PINS[0]=MSB, DIP_PINS[2]=LSB.
    Returns 3-character string '000' to '111', e.g. '010'.
    """
    bits = []
    for pin_name in DIP_PINS:
        pin = pyb.Pin(pin_name, pyb.Pin.IN, pyb.Pin.PULL_UP)
        # PULL_UP: switch closed = 0, open = 1
        bits.append('1' if pin.value() else '0')
    return ''.join(bits)


def main():
    """Dispatch to selected challenge based on DIP switch."""
    dip = read_dip_switch()
    
    if dip == '010':
        from challenge_5 import run_challenge_5
        run_challenge_5()
    elif dip == '100':
        from trajectory_tracking import run_autonomous_mode
        run_autonomous_mode()
    else:
        # Idle: show DIP value, optional placeholder for other challenges
        from oled_display import OLED
        oled = OLED()
        oled.clear()
        oled.draw_text(0, 0, "DIP: " + dip)
        oled.draw_text(0, 20, "Set to 010 for Bal.")
        oled.draw_text(0, 36, "Set to 100 for Auto")
        oled.display()
        while True:
            pyb.delay(500)


if __name__ == '__main__':
    main()
