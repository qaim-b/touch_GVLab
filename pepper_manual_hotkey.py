#!/usr/bin/env python3
"""
Manual hotkey controller for the Touch setup (NO ROS).

Use-case:
- Start with left arm "soft" (stiffness 0.0) so participant can position it.
  - Press '5' to lock (stiffness 1.0) and optionally squeeze (LHand close).
  - Press '0' to release (open hand + stiffness 0.0).
  - Press 'o' to open hand only.
  - Press 'q' to quit (cleanup).
"""

import argparse
import atexit
import sys
import termios
import time
import traceback
import tty
import os
from contextlib import redirect_stderr
import threading


# =========================
# Tunable experiment config
# =========================

PORT = 9559

# Left arm joints used for stiffness setting
LEFT_ARM_JOINTS = [
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
]

LEFT_HAND = "LHand"
LEFT_WRIST = "LWristYaw"

# Safety limits
SAFETY_HAND_MAX = 0.40

# Hotkey behavior
STIFFNESS_FREE = 0.05
# Make the wrist extra soft in FREE mode (0..1).
STIFFNESS_FREE_LWRISTYAW = 1.0
STIFFNESS_LOCK = 0.8
# Make the elbow joint stiffer than the rest when locked (0..1).
STIFFNESS_LOCK_LELBOWROLL = 1.0
# Keep wrist yaw soft even when locked (0..1).
STIFFNESS_LOCK_LWRISTYAW = 1.0
STIFFNESS_RAMP_S = 0.5

# Re-send current arm pose to help it hold position after changing stiffness.
HOLD_POSE_SPEED_FRACTION = 0.05

# Wrist holding: keep LWristYaw at a fixed angle for the whole session.
WRIST_HOLD_PERIOD_S = 0.2
WRIST_HOLD_SPEED_FRACTION = 0.08

# "Squeeze" when locking (set to 0.0 to disable hand squeeze)
LOCK_SQUEEZE_VALUE = 0.25
LOCK_SQUEEZE_SPEED_FRACTION = 0.20
LOCK_SQUEEZE_HOLD_S = 0.5

# Stronger/faster squish feel (shorter time, higher speed).
SQUISH_SMALL_CLOSE = 0.20
SQUISH_SMALL_SPEED_FRACTION = 0.45
SQUISH_SMALL_HOLD_S = 0.15

SQUISH_LARGE_CLOSE = 0.32
SQUISH_LARGE_SPEED_FRACTION = 0.60
SQUISH_LARGE_HOLD_S = 0.25

# NAOqi modules (best-effort disable to prevent automatic posture changes)
MOD_MOTION = "ALMotion"
MOD_AUTONOMOUS_LIFE = "ALAutonomousLife"
MOD_BASIC_AWARENESS = "ALBasicAwareness"


def _connect_naoqi(ip, port):
    """Connect to NAOqi via qi.Session (preferred) or naoqi.ALProxy (fallback)."""
    try:
        with open(os.devnull, "w") as _devnull, redirect_stderr(_devnull):
            import qi  # type: ignore

        # Create a qi.Application to avoid noisy qi.path warnings.
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


def _disable_autonomy(get_service):
    """Disable autonomous life and basic awareness (best-effort)."""
    try:
        auto = get_service(MOD_AUTONOMOUS_LIFE)
        try:
            auto.setState("disabled")
        except Exception:
            pass
    except Exception:
        pass
    try:
        aware = get_service(MOD_BASIC_AWARENESS)
        try:
            aware.stopAwareness()
        except Exception:
            try:
                aware.setEnabled(False)
            except Exception:
                pass
    except Exception:
        pass


def _set_left_arm_stiffness(motion, stiffness, duration_s):
    """Set left arm joint stiffnesses to a value (0..1)."""
    names = list(LEFT_ARM_JOINTS) + [LEFT_HAND]
    st = max(0.0, min(1.0, float(stiffness)))
    _nao_try("ALMotion.stiffnessInterpolation(LArm)", motion.stiffnessInterpolation, names, st, float(duration_s))


def _set_left_arm_lock_stiffness(motion, duration_s):
    """Set lock stiffness with extra stiffness on LElbowRoll."""
    names = list(LEFT_ARM_JOINTS) + [LEFT_HAND]
    stiffnesses = []
    for n in names:
        if n == "LElbowRoll":
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_LOCK_LELBOWROLL))))
        elif n == LEFT_WRIST:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_LOCK_LWRISTYAW))))
        else:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_LOCK))))
    _nao_try(
        "ALMotion.stiffnessInterpolation(lock profile)",
        motion.stiffnessInterpolation,
        names,
        stiffnesses,
        float(duration_s),
    )


def _set_left_arm_free_stiffness(motion, duration_s):
    """Set free stiffness with extra softness on LWristYaw."""
    names = list(LEFT_ARM_JOINTS) + [LEFT_HAND]
    stiffnesses = []
    for n in names:
        if n == LEFT_WRIST:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_FREE_LWRISTYAW))))
        else:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_FREE))))
    _nao_try(
        "ALMotion.stiffnessInterpolation(free profile)",
        motion.stiffnessInterpolation,
        names,
        stiffnesses,
        float(duration_s),
    )


def _open_hand(motion):
    """Open left hand fully."""
    _nao_try("ALMotion.setAngles(LHand open)", motion.setAngles, [LEFT_HAND], [0.0], float(LOCK_SQUEEZE_SPEED_FRACTION))


def _hold_current_left_arm_pose(motion):
    """Read current left arm sensor angles and re-send them to hold the current pose."""
    names = list(LEFT_ARM_JOINTS)
    angles = _nao_try("ALMotion.getAngles(LArm)", motion.getAngles, names, True)
    _nao_try(
        "ALMotion.setAngles(LArm hold)",
        motion.setAngles,
        names,
        angles,
        float(HOLD_POSE_SPEED_FRACTION),
    )


def _get_wrist_yaw(motion):
    """Read current LWristYaw (sensor)."""
    return float(_nao_try("ALMotion.getAngles(LWristYaw)", motion.getAngles, [LEFT_WRIST], True)[0])


def _hold_wrist_yaw(motion, wrist_angle_rad):
    """Command LWristYaw to a fixed angle."""
    _nao_try(
        "ALMotion.setAngles(LWristYaw hold)",
        motion.setAngles,
        [LEFT_WRIST],
        [float(wrist_angle_rad)],
        float(WRIST_HOLD_SPEED_FRACTION),
    )


def _start_wrist_hold_thread(motion, wrist_angle_rad, stop_event):
    """Start a background thread that keeps LWristYaw at a fixed angle."""

    def _loop():
        """Thread loop."""
        while not stop_event.is_set():
            try:
                _hold_wrist_yaw(motion, wrist_angle_rad)
            except Exception:
                pass
            time.sleep(float(WRIST_HOLD_PERIOD_S))

    t = threading.Thread(target=_loop, name="wrist_hold", daemon=True)
    t.start()
    return t


def _squeeze_hand(motion, close_value, speed_fraction, hold_s):
    """Close left hand to a safe value, hold, then open."""
    close = _clamp_hand(close_value)
    sp = max(0.01, min(1.0, float(speed_fraction)))
    _nao_try("ALMotion.setAngles(LHand close)", motion.setAngles, [LEFT_HAND], [close], sp)
    time.sleep(float(hold_s))
    _nao_try("ALMotion.setAngles(LHand open)", motion.setAngles, [LEFT_HAND], [0.0], sp)


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
    p = argparse.ArgumentParser(description="Manual hotkey control for Pepper Touch (left arm)")
    p.add_argument("--ip", required=True, help="Pepper IP address (e.g., 192.168.0.100)")
    p.add_argument("--port", type=int, default=PORT, help="NAOqi port (default: 9559)")
    return p.parse_args()


def main():
    """Entrypoint."""
    args = _parse_args()
    get_service = _connect_naoqi(args.ip, args.port)
    _disable_autonomy(get_service)
    motion = get_service(MOD_MOTION)

    wrist_hold_angle = _get_wrist_yaw(motion)
    wrist_stop = threading.Event()
    _start_wrist_hold_thread(motion, wrist_hold_angle, wrist_stop)

    def _cleanup():
        """Best-effort cleanup: open and free arm."""
        try:
            wrist_stop.set()
        except Exception:
            pass
        try:
            _open_hand(motion)
        except Exception:
            pass
        try:
            _set_left_arm_stiffness(motion, STIFFNESS_FREE, STIFFNESS_RAMP_S)
        except Exception:
            pass

    atexit.register(_cleanup)

    print("Hotkeys: '5'=LOCK(+squeeze), '0'=FREE, 'o'=open, 'q'=quit")
    print("Starting in FREE mode (stiffness=0.0). Participant can position the left arm now.")
    _set_left_arm_free_stiffness(motion, STIFFNESS_RAMP_S)
    _hold_current_left_arm_pose(motion)
    _open_hand(motion)

    with _RawKeyReader() as r:
        while True:
            k = r.read_key()
            if k == "q":
                break
            if k == "0":
                print("\nFREE: opening hand + stiffness=0.0")
                _open_hand(motion)
                _set_left_arm_free_stiffness(motion, STIFFNESS_RAMP_S)
                _hold_current_left_arm_pose(motion)
                continue
            if k == "o":
                print("\nOPEN: opening hand only")
                _open_hand(motion)
                continue
            if k == "5":
                print("\nLOCK: stiffness=1.0")
                _set_left_arm_lock_stiffness(motion, STIFFNESS_RAMP_S)
                _hold_current_left_arm_pose(motion)
                if LOCK_SQUEEZE_VALUE > 0.0:
                    print("LOCK: squeeze (LHand=%.3f, hold=%.2fs)" % (_clamp_hand(LOCK_SQUEEZE_VALUE), LOCK_SQUEEZE_HOLD_S))
                    _squeeze_hand(
                        motion,
                        close_value=LOCK_SQUEEZE_VALUE,
                        speed_fraction=LOCK_SQUEEZE_SPEED_FRACTION,
                        hold_s=LOCK_SQUEEZE_HOLD_S,
                    )
                continue
            if k == "x":
                print('\nSMALL squish (LHand=%.3f, hold=%.2fs)' % (_clamp_hand(SQUISH_SMALL_CLOSE), SQUISH_SMALL_HOLD_S))
                _squeeze_hand(
                    motion,
                    close_value=SQUISH_SMALL_CLOSE,
                    speed_fraction=SQUISH_SMALL_SPEED_FRACTION,
                    hold_s=SQUISH_SMALL_HOLD_S,
                )
                continue
            if k == "y":
                print('\nLARGE squish (LHand=%.3f, hold=%.2fs)' % (_clamp_hand(SQUISH_LARGE_CLOSE), SQUISH_LARGE_HOLD_S))
                _squeeze_hand(
                    motion,
                    close_value=SQUISH_LARGE_CLOSE,
                    speed_fraction=SQUISH_LARGE_SPEED_FRACTION,
                    hold_s=SQUISH_LARGE_HOLD_S,
                )
                continue

    _cleanup()
    print("\nExited.")


if __name__ == "__main__":
    main()
