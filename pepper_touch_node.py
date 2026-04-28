#!/usr/bin/env python3
"""
ROS node: Stroop-triggered Pepper "Touch" (left-hand squish).

Purpose:
- Subscribe to a Stroop end-of-round topic.
- Trigger a "small" or "large" squish on Pepper's left hand.
- Keep Pepper silent/still (best-effort) and log each round to CSV.

This is designed for ROS Noetic (Python 3).
"""

import csv
import datetime as _dt
import os
import sys
import time
import traceback
from contextlib import redirect_stderr


# =========================
# Tunable experiment config
# =========================

PORT = 9559

CSV_PATH = "squish_log.csv"

DEFAULT_ARM_ANGLES = {
    "LShoulderPitch": 0.60,
    "LShoulderRoll": 0.20,
    "LElbowYaw": -1.10,
    "LElbowRoll": -0.60,
    # 90 degrees (pi/2) wrist yaw
    "LWristYaw": 1.5708,
}

REST_ARM_ANGLES = dict(DEFAULT_ARM_ANGLES)

SAFETY_HAND_MAX = 0.40
SAFETY_JOINT_VEL_MAX_RAD_S = 0.5

SQUISH_PROFILES = {
    "small": {"close": 0.15, "speed": 0.15, "hold_s": 0.3},
    "large": {"close": 0.35, "speed": 0.25, "hold_s": 0.6},
}

# NAOqi modules
MOD_AUTONOMOUS_LIFE = "ALAutonomousLife"
MOD_BASIC_AWARENESS = "ALBasicAwareness"
MOD_LEDS = "ALLeds"
MOD_MOTION = "ALMotion"


def _iso_now():
    """Return current timestamp in ISO-8601 (local time)."""
    return _dt.datetime.now().astimezone().isoformat(timespec="milliseconds")


def _connect_naoqi(ip, port):
    """Connect to NAOqi via qi.Session (preferred) or naoqi.ALProxy (fallback)."""
    try:
        with open(os.devnull, "w") as _devnull, redirect_stderr(_devnull):
            import qi  # type: ignore

        app = qi.Application(["pepper_touch_node", "--qi-url=tcp://%s:%d" % (ip, int(port))])
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
            return ALProxy(name, ip, int(port))

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
        print("[WARN] Could not disable ALAutonomousLife (continuing). Error: %s" % exc, file=sys.stderr)

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
    """Open LHand fully."""
    sf = max(0.01, min(1.0, float(speed_fraction)))
    _nao_try("ALMotion.setAngles(LHand=0.0)", motion.setAngles, ["LHand"], [0.0], sf)


def _move_left_arm(motion, arm_angles):
    """Move left arm joints to target angles after opening hand (safety rule)."""
    joint_names = list(arm_angles.keys())
    target = [float(arm_angles[j]) for j in joint_names]
    safe_fraction = _compute_safe_fraction(motion, joint_names)
    _open_hand(motion, speed_fraction=safe_fraction)
    _nao_try("ALMotion.setAngles(LArm joints)", motion.setAngles, joint_names, target, safe_fraction)


def _execute_squish(motion, squish_type):
    """Execute a squish profile on LHand (close-hold-open) with safety clamping."""
    if squish_type not in SQUISH_PROFILES:
        raise ValueError("Unknown squish_type: %r" % (squish_type,))
    profile = SQUISH_PROFILES[squish_type]
    close = float(profile["close"])
    hold_s = float(profile["hold_s"])
    speed = float(profile["speed"])

    if close > SAFETY_HAND_MAX:
        raise ValueError("Requested hand close %.3f exceeds SAFETY_HAND_MAX %.3f" % (close, SAFETY_HAND_MAX))

    speed_fraction = max(0.01, min(1.0, speed))
    _nao_try("ALMotion.setAngles(LHand close)", motion.setAngles, ["LHand"], [close], speed_fraction)
    time.sleep(hold_s)
    _nao_try("ALMotion.setAngles(LHand open)", motion.setAngles, ["LHand"], [0.0], speed_fraction)
    return close, hold_s


def _init_csv(path):
    """Create CSV file with a header and return open handle + writer."""
    f = open(path, "a", newline="")
    w = csv.writer(f)
    if f.tell() == 0:
        w.writerow(
            [
                "timestamp_iso",
                "round_number",
                "squish_type",
                "hand_close_value",
                "hold_duration_s",
                "success",
            ]
        )
        f.flush()
    return f, w


def _parse_squish_from_payload(payload, default_type):
    """Best-effort mapping from incoming message payload to squish type."""
    if payload is None:
        return default_type
    if isinstance(payload, bool):
        # Bool messages typically signal "end of round". Always trigger.
        return default_type

    s = str(payload).strip().lower()
    if not s:
        return default_type
    if "small" in s:
        return "small"
    if "large" in s:
        return "large"
    if s in ("0", "false", "f", "no"):
        return "small"
    if s in ("1", "true", "t", "yes"):
        return "large"
    return default_type


def main():
    """Entrypoint."""
    try:
        import rospy  # type: ignore
        from std_msgs.msg import Bool, String  # type: ignore
    except Exception as exc:
        raise SystemExit(
            "Failed to import ROS packages. Source ROS Noetic first, e.g.: "
            "`source /opt/ros/noetic/setup.bash` (then rerun). Error: %s" % exc
        )

    rospy.init_node("pepper_touch_node", anonymous=False)

    pepper_ip = rospy.get_param("~ip", None)
    if not pepper_ip:
        raise SystemExit("Missing required ROS param: ~ip (Pepper IP)")
    pepper_port = int(rospy.get_param("~port", PORT))

    trigger_topic = rospy.get_param("~trigger_topic", "/stroop/answer")
    msg_type = str(rospy.get_param("~msg_type", "String")).strip()

    init_pose = bool(rospy.get_param("~init_pose", True))
    rest_pose_on_shutdown = bool(rospy.get_param("~rest_pose_on_shutdown", True))
    open_hand_on_shutdown = bool(rospy.get_param("~open_hand_on_shutdown", True))

    # Squish selection:
    # - If payload is parseable, it can override.
    # - Otherwise the node uses a repeating sequence.
    squish_sequence = rospy.get_param("~squish_sequence", ["small", "large"])
    if not isinstance(squish_sequence, list) or not squish_sequence:
        squish_sequence = ["small", "large"]

    csv_path = rospy.get_param("~csv_path", CSV_PATH)

    get_service, _info = _connect_naoqi(pepper_ip, pepper_port)
    motion = get_service(MOD_MOTION)
    prev_state = _set_autonomous_silent(get_service)

    csv_f, csv_w = _init_csv(csv_path)
    round_number = 0
    seq_idx = 0

    def _shutdown():
        """ROS shutdown hook (best-effort)."""
        try:
            if open_hand_on_shutdown:
                _open_hand(motion, speed_fraction=0.1)
        except Exception:
            pass
        try:
            if rest_pose_on_shutdown:
                _move_left_arm(motion, REST_ARM_ANGLES)
        except Exception:
            pass
        try:
            _restore_autonomous(get_service, prev_state)
        except Exception:
            pass
        try:
            csv_f.flush()
            csv_f.close()
        except Exception:
            pass

    rospy.on_shutdown(_shutdown)

    if init_pose:
        rospy.loginfo("Moving to DEFAULT_ARM_ANGLES...")
        _move_left_arm(motion, DEFAULT_ARM_ANGLES)

    rospy.loginfo("Pepper touch node ready.")
    rospy.loginfo("Pepper: %s:%d | topic: %s (%s)", pepper_ip, pepper_port, trigger_topic, msg_type)
    rospy.loginfo("CSV: %s", csv_path)

    def _trigger(payload):
        nonlocal round_number, seq_idx
        if rospy.is_shutdown():
            return

        default_type = str(squish_sequence[seq_idx % len(squish_sequence)]).strip().lower() or "small"
        if default_type not in SQUISH_PROFILES:
            default_type = "small"

        squish_type = _parse_squish_from_payload(payload, default_type)
        seq_idx += 1
        round_number += 1

        success = False
        close_value = ""
        hold_s = ""
        try:
            close_value, hold_s = _execute_squish(motion, squish_type)
            success = True
            rospy.loginfo("Round %d: squish=%s (close=%.3f hold=%.2fs)", round_number, squish_type, close_value, hold_s)
        except Exception as exc:
            rospy.logerr("Round %d FAILED: squish=%s error=%s", round_number, squish_type, exc)
        finally:
            try:
                csv_w.writerow([_iso_now(), int(round_number), str(squish_type), close_value, hold_s, bool(success)])
                csv_f.flush()
            except Exception:
                pass

    if msg_type.lower() == "bool":

        def _cb(msg):
            if bool(getattr(msg, "data", False)):
                _trigger(True)

        rospy.Subscriber(trigger_topic, Bool, _cb, queue_size=10)
    else:

        def _cb(msg):
            _trigger(getattr(msg, "data", ""))

        rospy.Subscriber(trigger_topic, String, _cb, queue_size=10)

    rospy.spin()


if __name__ == "__main__":
    main()

