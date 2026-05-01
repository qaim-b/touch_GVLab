#!/usr/bin/env python3
"""Pepper Touch verification script (NO ROS)."""

import argparse
import csv
import datetime as _dt
import sys
import time
import traceback
import os
from contextlib import redirect_stderr

# =========================
# Tunable experiment config
# =========================

CSV_TEST_PATH = "squish_log_test.csv"

DEFAULT_ARM_ANGLES = {
    "RShoulderPitch": 0.60,
    "RShoulderRoll": -0.20,
    "RElbowYaw": 1.10,
    "RElbowRoll": 0.60,
    # 90 degrees (pi/2) wrist yaw
    "RWristYaw": 1.5708,
}

REST_ARM_ANGLES = dict(DEFAULT_ARM_ANGLES)

SAFETY_HAND_MAX = 0.40
SAFETY_JOINT_VEL_MAX_RAD_S = 0.5

SQUISH_PROFILES = {
    "small": {"close": 0.15, "speed": 0.15, "hold_s": 0.3},
    "large": {"close": 0.35, "speed": 0.25, "hold_s": 0.6},
}

MOD_AUTONOMOUS_LIFE = "ALAutonomousLife"
MOD_BASIC_AWARENESS = "ALBasicAwareness"
MOD_LEDS = "ALLeds"
MOD_MOTION = "ALMotion"


def _iso_now():
    """Return current timestamp in ISO-8601 (local time)."""
    return _dt.datetime.now().astimezone().isoformat(timespec="milliseconds")


def _connect_naoqi(ip):
    """Connect to NAOqi via qi.Session (preferred) or naoqi.ALProxy (fallback)."""
    try:
        with open(os.devnull, "w") as _devnull, redirect_stderr(_devnull):
            import qi  # type: ignore

        # Create a qi.Application to avoid noisy qi.path warnings.
        app = qi.Application(["pepper_test", "--qi-url=tcp://%s:9559" % ip])
        app.start()
        session = app.session

        def get_service(name):
            return session.service(name)

        return get_service, {"backend": "qi", "session": session}
    except Exception:
        pass

    try:
        from naoqi import ALProxy  # type: ignore

        def get_service(name):
            return ALProxy(name, ip, 9559)

        return get_service, {"backend": "naoqi", "session": None}
    except Exception as exc:
        raise RuntimeError("Failed to import/connect NAOqi (qi or naoqi).") from exc


def _nao_try(desc, fn, *args, **kwargs):
    """Call NAOqi function with error handling; re-raise after printing context."""
    try:
        return fn(*args, **kwargs)
    except Exception as exc:
        print("[NAOqi ERROR] %s: %s" % (desc, exc), file=sys.stderr)
        traceback.print_exc()
        raise


def _set_autonomous_silent(get_service):
    """Disable autonomous life, awareness, and LED animations (best-effort)."""
    auto = get_service(MOD_AUTONOMOUS_LIFE)
    aware = get_service(MOD_BASIC_AWARENESS)
    leds = get_service(MOD_LEDS)

    prev_state = None
    try:
        prev_state = _nao_try("ALAutonomousLife.getState", auto.getState)
    except Exception:
        prev_state = None

    try:
        _nao_try("ALAutonomousLife.setState(disabled)", auto.setState, "disabled")
    except Exception as exc:
        # Pepper can refuse setState if onboarding/getting-started wizard isn't completed.
        # Continue best-effort so we can still run motion tests.
        print(
            "[WARN] Could not disable ALAutonomousLife (continuing). Error: %s" % exc,
            file=sys.stderr,
        )

    try:
        _nao_try("ALBasicAwareness.stopAwareness", aware.stopAwareness)
    except Exception:
        try:
            _nao_try("ALBasicAwareness.setEnabled(False)", aware.setEnabled, False)
        except Exception:
            pass

    try:
        _nao_try("ALLeds.off(All)", leds.off, "All")
    except Exception:
        pass

    return prev_state


def _restore_autonomous(get_service, prev_state):
    """Restore autonomous life state (best-effort)."""
    auto = get_service(MOD_AUTONOMOUS_LIFE)
    target = prev_state or "solitary"
    try:
        _nao_try("ALAutonomousLife.setState(%s)" % target, auto.setState, target)
    except Exception:
        pass


def _compute_safe_fraction(motion, joint_names):
    """Compute a safe fractionMaxSpeed to keep joint velocity <= SAFETY_JOINT_VEL_MAX_RAD_S."""
    fractions = []
    for j in joint_names:
        try:
            limits = _nao_try("ALMotion.getLimits(%s)" % j, motion.getLimits, j)
            max_vel = float(limits[0][2])
            if max_vel <= 0.0:
                continue
            fractions.append(SAFETY_JOINT_VEL_MAX_RAD_S / max_vel)
        except Exception:
            continue
    if not fractions:
        return 0.1
    return max(0.01, min(1.0, min(fractions)))


def _open_hand(motion, speed_fraction):
    """Open RHand fully."""
    sf = max(0.01, min(1.0, float(speed_fraction)))
    _nao_try("ALMotion.setAngles(RHand=0.0)", motion.setAngles, ["RHand"], [0.0], sf)


def _move_right_arm(motion, arm_angles):
    """Move right arm joints to target angles after opening hand (safety rule)."""
    joint_names = list(arm_angles.keys())
    target = [float(arm_angles[j]) for j in joint_names]
    safe_fraction = _compute_safe_fraction(motion, joint_names)
    _open_hand(motion, speed_fraction=safe_fraction)
    _nao_try("ALMotion.setAngles(RArm joints)", motion.setAngles, joint_names, target, safe_fraction)


def _execute_squish(motion, squish_type):
    """Execute a squish profile on RHand (close-hold-open) with safety clamping."""
    profile = SQUISH_PROFILES[squish_type]
    close = float(profile["close"])
    hold_s = float(profile["hold_s"])
    speed = float(profile["speed"])

    if close > SAFETY_HAND_MAX:
        raise ValueError("Requested hand close %.3f exceeds SAFETY_HAND_MAX %.3f" % (close, SAFETY_HAND_MAX))

    speed_fraction = max(0.01, min(1.0, speed))

    _nao_try("ALMotion.setAngles(RHand close)", motion.setAngles, ["RHand"], [close], speed_fraction)
    time.sleep(hold_s)
    try:
        current = _nao_try("ALMotion.getAngles(RHand)", motion.getAngles, ["RHand"], True)[0]
        print("RHand during hold: %.3f" % float(current))
    except Exception:
        pass
    _nao_try("ALMotion.setAngles(RHand open)", motion.setAngles, ["RHand"], [0.0], speed_fraction)

    return close, hold_s


def _init_csv(path):
    """Create CSV file with a header and return open handle."""
    f = open(path, "a", newline="")
    w = csv.writer(f)
    if f.tell() == 0:
        w.writerow(["timestamp_iso", "note", "success"])
        f.flush()
    return f, w


def _parse_args():
    """Parse CLI arguments."""
    p = argparse.ArgumentParser(description="Pepper Touch verification (no ROS)")
    p.add_argument("--ip", required=True, help="Pepper IP address (e.g., 192.168.1.10)")
    p.add_argument(
        "--pause",
        action="store_true",
        help="Pause between phases (press ENTER to continue).",
    )
    return p.parse_args()

def _pause_if_enabled(enabled, message):
    """Pause for ENTER if enabled and stdin is a TTY."""
    if not enabled:
        return
    if not sys.stdin.isatty():
        # When running non-interactively (e.g., via an executor), don't block forever.
        time.sleep(1.0)
        return
    input(message)


def main():
    """Entrypoint."""
    args = _parse_args()

    get_service, _info = _connect_naoqi(args.ip)
    motion = get_service(MOD_MOTION)
    prev_state = _set_autonomous_silent(get_service)

    csv_f, csv_w = _init_csv(CSV_TEST_PATH)
    success = False

    try:
        _pause_if_enabled(args.pause, "Ready to move to DEFAULT_ARM_ANGLES. Press ENTER...")
        _move_right_arm(motion, DEFAULT_ARM_ANGLES)
        try:
            joint_names = list(DEFAULT_ARM_ANGLES.keys())
            actual = _nao_try("ALMotion.getAngles(RArm)", motion.getAngles, joint_names, True)
            print("Actual right-arm angles:")
            for name, val in zip(joint_names, actual):
                print("  %s: %.3f" % (name, float(val)))
        except Exception:
            pass

        _pause_if_enabled(args.pause, "Ready for SMALL squish. Press ENTER...")
        time.sleep(0.2)

        close, hold_s = _execute_squish(motion, "small")
        print('Executed "small" squish (close=%.3f, hold_s=%.2f)' % (close, hold_s))
        _pause_if_enabled(args.pause, "Ready for LARGE squish. Press ENTER...")
        time.sleep(0.2)

        close, hold_s = _execute_squish(motion, "large")
        print('Executed "large" squish (close=%.3f, hold_s=%.2f)' % (close, hold_s))
        _pause_if_enabled(args.pause, "Ready to return to REST pose. Press ENTER...")
        time.sleep(0.2)

        _open_hand(motion, speed_fraction=0.1)
        _move_right_arm(motion, REST_ARM_ANGLES)
        success = True
    finally:
        try:
            _restore_autonomous(get_service, prev_state)
        except Exception:
            pass
        csv_w.writerow([_iso_now(), "pepper_test_complete", bool(success)])
        csv_f.flush()
        csv_f.close()

    print("Wrote CSV test log: %s" % CSV_TEST_PATH)


if __name__ == "__main__":
    main()
