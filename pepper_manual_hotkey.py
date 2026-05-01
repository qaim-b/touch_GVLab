#!/usr/bin/env python3
"""
Manual hotkey controller for the Touch setup (NO ROS).

Use-case:
- Start with right arm "soft" (stiffness 0.0) so participant can position it.
  - Press '5' to lock (stiffness 1.0) and optionally squeeze (RHand close).
  - Press '0' to release (open hand + stiffness 0.0).
  - Press 'o' to open hand only.
  - Press 'q' to quit (cleanup).
"""

import argparse
import atexit
import os
import sys
import termios
import time
import traceback
import tty
from contextlib import redirect_stderr


# =========================
# Tunable experiment config
# =========================

PORT = 9559

# Right arm joints used for stiffness setting
RIGHT_ARM_JOINTS = [
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
]

RIGHT_HAND = "RHand"

# Safety limits
SAFETY_HAND_MAX = 0.40

# Hotkey behavior
STIFFNESS_FREE = 0.0
STIFFNESS_LOCK = 1.0 
STIFFNESS_RAMP_S = 0.5

# "Squeeze" when locking (set to 0.0 to disable hand squeeze)
LOCK_SQUEEZE_VALUE = 0.25
LOCK_SQUEEZE_SPEED_FRACTION = 0.20
LOCK_SQUEEZE_HOLD_S = 0.5


def _connect_naoqi(ip, port):
    """Connect to NAOqi via qi.Session (preferred) or naoqi.ALProxy (fallback)."""
    try:
        with open(os.devnull, "w") as _devnull, redirect_stderr(_devnull):
            import qi  # type: ignore

        app = qi.Application(["pepper_manual_hotkey", "--qi-url=tcp://%s:%d" % (ip, int(port))])
        app.start()
        session = app.session

        def get_service(name):
            return session.service(name)

        return get_service
    except Exception:
        pass

    from naoqi import ALProxy  # type: ignore

    def get_service(name):
        return ALProxy(name, ip, int(port))

    return get_service


def _nao_try(desc, fn, *args, **kwargs):
    """Call NAOqi function with error handling; re-raise after printing context."""
    try:
        return fn(*args, **kwargs)
    except Exception as exc:
        print("[NAOqi ERROR] %s: %s" % (desc, exc), file=sys.stderr)
        traceback.print_exc()
        raise


def _clamp_hand(value):
    """Clamp hand close value to safety maximum."""
    v = float(value)
    if v < 0.0:
        v = 0.0
    if v > SAFETY_HAND_MAX:
        v = SAFETY_HAND_MAX
    return v


def _set_right_arm_stiffness(motion, stiffness, duration_s):
    """Set right arm joint stiffnesses to a value (0..1)."""
    names = list(RIGHT_ARM_JOINTS) + [RIGHT_HAND]
    st = max(0.0, min(1.0, float(stiffness)))
    _nao_try("ALMotion.stiffnessInterpolation(RArm)", motion.stiffnessInterpolation, names, st, float(duration_s))


def _open_hand(motion):
    """Open right hand fully."""
    _nao_try("ALMotion.setAngles(RHand open)", motion.setAngles, [RIGHT_HAND], [0.0], float(LOCK_SQUEEZE_SPEED_FRACTION))


def _squeeze_hand(motion, close_value, speed_fraction, hold_s):
    """Close right hand to a safe value, hold, then open."""
    close = _clamp_hand(close_value)
    sp = max(0.01, min(1.0, float(speed_fraction)))
    _nao_try("ALMotion.setAngles(RHand close)", motion.setAngles, [RIGHT_HAND], [close], sp)
    time.sleep(float(hold_s))
    _nao_try("ALMotion.setAngles(RHand open)", motion.setAngles, [RIGHT_HAND], [0.0], sp)


class _RawKeyReader:
    """Context manager to read single keypresses from stdin."""

    def __init__(self):
        """Initialize with no-op defaults."""
        self._fd = None
        self._old = None

    def __enter__(self):
        """Enter raw mode."""
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        """Restore terminal mode."""
        if self._fd is not None and self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read_key(self):
        """Read one character."""
        return sys.stdin.read(1)


def _parse_args():
    """Parse CLI arguments."""
    p = argparse.ArgumentParser(description="Manual hotkey control for Pepper Touch (right arm)")
    p.add_argument("--ip", required=True, help="Pepper IP address (e.g., 192.168.0.100)")
    p.add_argument("--port", type=int, default=PORT, help="NAOqi port (default: 9559)")
    return p.parse_args()


def main():
    """Entrypoint."""
    args = _parse_args()
    get_service = _connect_naoqi(args.ip, args.port)
    motion = get_service("ALMotion")
    try:
        motion.wakeUp()
    except Exception:
        pass

    def _cleanup():
        """Best-effort cleanup: open and free arm."""
        try:
            _open_hand(motion)
        except Exception:
            pass
        try:
            _set_right_arm_stiffness(motion, STIFFNESS_FREE, STIFFNESS_RAMP_S)
        except Exception:
            pass

    atexit.register(_cleanup)

    print("Hotkeys: '5'=LOCK(+squeeze), '0'=FREE, 'o'=open, 'q'=quit")
    print("Starting in FREE mode (stiffness=0.0). Participant can position the right arm now.")
    _set_right_arm_stiffness(motion, STIFFNESS_FREE, STIFFNESS_RAMP_S)
    _open_hand(motion)

    with _RawKeyReader() as r:
        while True:
            k = r.read_key()
            if k == "q":
                break
            if k == "0":
                print("\nFREE: opening hand + stiffness=0.0")
                _open_hand(motion)
                _set_right_arm_stiffness(motion, STIFFNESS_FREE, STIFFNESS_RAMP_S)
                continue
            if k == "o":
                print("\nOPEN: opening hand only")
                _open_hand(motion)
                continue
            if k == "5":
                print("\nLOCK: stiffness=1.0")
                _set_right_arm_stiffness(motion, STIFFNESS_LOCK, STIFFNESS_RAMP_S)
                if LOCK_SQUEEZE_VALUE > 0.0:
                    print("LOCK: squeeze (RHand=%.3f, hold=%.2fs)" % (_clamp_hand(LOCK_SQUEEZE_VALUE), LOCK_SQUEEZE_HOLD_S))
                    _squeeze_hand(
                        motion,
                        close_value=LOCK_SQUEEZE_VALUE,
                        speed_fraction=LOCK_SQUEEZE_SPEED_FRACTION,
                        hold_s=LOCK_SQUEEZE_HOLD_S,
                    )
                continue

    _cleanup()
    print("\nExited.")


if __name__ == "__main__":
    main()
