#!/usr/bin/env python3
"""
Xbox controller operator for Pepper Touch (NO ROS, no extra deps).

Reads Linux joystick events from /dev/input/js0 and sends NAOqi commands.

Default mapping:
- A button: lock stiffness + squeeze once
- B button: free stiffness + open hand
- X button: small squish
- Y button: large squish
- START button: quit
"""

import argparse
import atexit
import math
import os
import struct
import sys
import time
import traceback
from contextlib import redirect_stderr
import threading


# =========================
# Tunable operator config
# =========================

PORT = 9559
JOYSTICK_DEV = "/dev/input/js0"

# Arm joint sets (Pepper)
LEFT_ARM_JOINTS = [
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
]
LEFT_HAND = "LHand"
LEFT_WRIST = "LWristYaw"

RIGHT_ARM_JOINTS = [
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
]
RIGHT_HAND = "RHand"
RIGHT_WRIST = "RWristYaw"

STIFFNESS_FREE = 0.0
STIFFNESS_LOCK = 1.0
STIFFNESS_RAMP_S = 0.5
STIFFNESS_LOCK_LWRISTYAW = 1.0
# Wrist should not move at all (hold current angle).
STIFFNESS_FREE_LWRISTYAW = 1.0

# "Float/hold" stiffness: enough to keep the arm up in the air while still feeling safe.
# This is what you want after the participant positions the arm.
STIFFNESS_HOLD = 0.7
STIFFNESS_HOLD_LWRISTYAW = 1.0

# Re-send current arm pose to help it hold position after changing stiffness.
HOLD_POSE_SPEED_FRACTION = 0.05

# Wrist holding: keep LWristYaw at a fixed angle for the whole session.
WRIST_HOLD_PERIOD_S = 0.2
WRIST_HOLD_SPEED_FRACTION = 0.08
WRIST_HOLD_ENABLED = True

# RT auto-placement (right arm) - tune on hardware
RT_AUTOPLACE_ENABLED = True
RT_AUTOPLACE_SPEED_FRACTION = 0.18
RT_AUTOPLACE_ELBOW_CONTRACT = 1.30  # radians (bent first to avoid table collision)

# Table rest pose (right arm) - tune these
RIGHT_TABLE_POSE_ANGLES = {
    "RShoulderPitch": 0.80,
    "RShoulderRoll": -0.25,
    "RElbowYaw": 1.10,
    "RElbowRoll": 0.60,
    "RWristYaw": 1.57,
}

# Safety
SAFETY_HAND_MAX = 0.40
SAFETY_DANCE_SPEED_FRACTION = 0.8

# Squish profiles (operator-triggered)
SQUISH_SMALL_CLOSE = 0.20
SQUISH_SMALL_SPEED = 0.45
SQUISH_SMALL_HOLD_S = 0.15

SQUISH_LARGE_CLOSE = 0.32
SQUISH_LARGE_SPEED = 0.60
SQUISH_LARGE_HOLD_S = 0.25

# Y-button "hard" squish (clamped by SAFETY_HAND_MAX)
SQUISH_HARD_CLOSE = SAFETY_HAND_MAX
SQUISH_HARD_SPEED = 1.0
SQUISH_HARD_HOLD_S = 0.35

# Lock squeeze (when pressing A)
LOCK_SQUEEZE_VALUE = 0.25
LOCK_SQUEEZE_SPEED = 0.20
LOCK_SQUEEZE_HOLD_S = 0.5

# NAOqi modules (best-effort disable to prevent automatic posture changes)
MOD_MOTION = "ALMotion"
MOD_AUTONOMOUS_LIFE = "ALAutonomousLife"
MOD_BASIC_AWARENESS = "ALBasicAwareness"
MOD_TTS = "ALTextToSpeech"
MOD_POSTURE = "ALRobotPosture"

# Speech lines (customizable)
SAY_INTRO = (
    "Yo, welcome to the Stroop game! Put your arm on that marked spot, say the color you see out loud, "
    "and don't move, aight."
)
SAY_BETWEEN_ROUNDS = (
    "Aight that round is DONE. Take a break homie, you look like you need it. "
    "We coming back tho, don't get too comfortable."
)
SAY_LAST_ROUND = "Yo that was the LAST round! How you feelin? You crushed it or did it crush you? Real talk tho, you did that. Respect."
INTRO_PAUSE_S = 3.0

# Linux joystick event format:
# struct js_event { u32 time; s16 value; u8 type; u8 number; }
_JS_EVENT_STRUCT = struct.Struct("IhBB")
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80


def _connect_naoqi(ip, port):
    """Connect to NAOqi via qi.Session (preferred) or naoqi.ALProxy (fallback)."""
    try:
        # qi prints a noisy "No Application was created..." warning on import in some installs.
        # Silence stderr for the import only.
        with open(os.devnull, "w") as _devnull, redirect_stderr(_devnull):
            import qi  # type: ignore

        # Create a qi.Application to avoid noisy qi.path warnings.
        app = qi.Application(["pepper_xbox_operator", "--qi-url=tcp://%s:%d" % (ip, int(port))])
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


def _clamp_hand(v):
    """Clamp hand value to [0, SAFETY_HAND_MAX]."""
    x = float(v)
    if x < 0.0:
        x = 0.0
    if x > SAFETY_HAND_MAX:
        x = SAFETY_HAND_MAX
    return x


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


def _wake_up(motion):
    """Wake up motors (best-effort) so stiffness commands take effect."""
    try:
        motion.wakeUp()
    except Exception:
        pass


def _set_stiffness(motion, arm_joints, hand_joint, stiffness):
    """Set arm + hand stiffness."""
    names = list(arm_joints) + [hand_joint]
    st = max(0.0, min(1.0, float(stiffness)))
    _nao_try("ALMotion.stiffnessInterpolation(LArm)", motion.stiffnessInterpolation, names, st, STIFFNESS_RAMP_S)


def _set_lock_stiffness(motion, arm_joints, hand_joint, wrist_joint):
    """Lock stiffness and keep wrist yaw fixed."""
    names = list(arm_joints) + [hand_joint]
    stiffnesses = []
    for n in names:
        if n == wrist_joint:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_LOCK_LWRISTYAW))))
        else:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_LOCK))))
    _nao_try(
        "ALMotion.stiffnessInterpolation(lock profile)",
        motion.stiffnessInterpolation,
        names,
        stiffnesses,
        STIFFNESS_RAMP_S,
    )

def _set_hold_stiffness(motion, arm_joints, hand_joint, wrist_joint):
    """Hold stiffness (keeps arm up)."""
    names = list(arm_joints) + [hand_joint]
    stiffnesses = []
    for n in names:
        if n == wrist_joint:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_HOLD_LWRISTYAW))))
        else:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_HOLD))))
    _nao_try(
        "ALMotion.stiffnessInterpolation(hold profile)",
        motion.stiffnessInterpolation,
        names,
        stiffnesses,
        STIFFNESS_RAMP_S,
    )


def _set_free_stiffness(motion, arm_joints, hand_joint, wrist_joint):
    """Free stiffness (wrist can optionally differ)."""
    names = list(arm_joints) + [hand_joint]
    stiffnesses = []
    for n in names:
        if n == wrist_joint:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_FREE_LWRISTYAW))))
        else:
            stiffnesses.append(max(0.0, min(1.0, float(STIFFNESS_FREE))))
    _nao_try(
        "ALMotion.stiffnessInterpolation(free profile)",
        motion.stiffnessInterpolation,
        names,
        stiffnesses,
        STIFFNESS_RAMP_S,
    )


def _hold_current_arm_pose(motion, arm_joints):
    """Read current arm sensor angles and re-send them to hold the current pose."""
    names = list(arm_joints)
    angles = _nao_try("ALMotion.getAngles(Arm)", motion.getAngles, names, True)
    _nao_try(
        "ALMotion.setAngles(Arm hold)",
        motion.setAngles,
        names,
        angles,
        float(HOLD_POSE_SPEED_FRACTION),
    )


def _start_pose_hold_thread(motion, joint_names, stop_event):
    """Start a thread that re-sends the current pose to hold it."""
    target = _nao_try("ALMotion.getAngles(pose target)", motion.getAngles, list(joint_names), True)

    def _loop():
        """Thread loop."""
        while not stop_event.is_set():
            try:
                _nao_try(
                    "ALMotion.setAngles(pose hold)",
                    motion.setAngles,
                    list(joint_names),
                    target,
                    float(HOLD_POSE_SPEED_FRACTION),
                )
            except Exception:
                pass
            time.sleep(0.1)

    t = threading.Thread(target=_loop, name="pose_hold", daemon=True)
    t.start()
    return t


def _get_wrist_yaw(motion, wrist_joint):
    """Read current wrist yaw (sensor)."""
    return float(_nao_try("ALMotion.getAngles(WristYaw)", motion.getAngles, [wrist_joint], True)[0])


def _hold_wrist_yaw(motion, wrist_joint, wrist_angle_rad):
    """Command wrist yaw to a fixed angle."""
    _nao_try(
        "ALMotion.setAngles(LWristYaw hold)",
        motion.setAngles,
        [wrist_joint],
        [float(wrist_angle_rad)],
        float(WRIST_HOLD_SPEED_FRACTION),
    )


def _autoplace_right_arm_to_table(motion):
    """Move right arm onto the table safely (contract elbow first), then lock + hold pose."""
    if not RT_AUTOPLACE_ENABLED:
        return
    _wake_up(motion)
    # Step 1: contract elbow to lift hand above the table plane
    _nao_try(
        "ALMotion.setAngles(RElbowRoll contract)",
        motion.setAngles,
        ["RElbowRoll"],
        [float(RT_AUTOPLACE_ELBOW_CONTRACT)],
        float(RT_AUTOPLACE_SPEED_FRACTION),
    )
    time.sleep(0.6)
    # Step 2: move to table pose
    names = list(RIGHT_TABLE_POSE_ANGLES.keys())
    targets = [float(RIGHT_TABLE_POSE_ANGLES[n]) for n in names]
    _nao_try(
        "ALMotion.setAngles(RIGHT_TABLE_POSE_ANGLES)",
        motion.setAngles,
        names,
        targets,
        float(RT_AUTOPLACE_SPEED_FRACTION),
    )


def _start_wrist_hold_thread(motion, wrist_joint, wrist_target_ref, stop_event):
    """Start a background thread that keeps LWristYaw at a (mutable) target angle.

    wrist_target_ref must be a dict-like object with key "angle" (radians).
    """

    def _loop():
        """Thread loop."""
        while not stop_event.is_set():
            try:
                _hold_wrist_yaw(motion, wrist_joint, float(wrist_target_ref.get("angle", 0.0)))
            except Exception:
                pass
            time.sleep(float(WRIST_HOLD_PERIOD_S))

    t = threading.Thread(target=_loop, name="wrist_hold", daemon=True)
    t.start()
    return t


def _open_hand(motion, hand_joint):
    """Open hand."""
    _nao_try("ALMotion.setAngles(Hand open)", motion.setAngles, [hand_joint], [0.0], 0.2)


def _squish(motion, hand_joint, close, speed, hold_s):
    """Close-hold-open on hand (safe clamp)."""
    c = _clamp_hand(close)
    sp = max(0.01, min(1.0, float(speed)))
    _nao_try("ALMotion.setAngles(Hand close)", motion.setAngles, [hand_joint], [c], sp)
    time.sleep(float(hold_s))
    _nao_try("ALMotion.setAngles(Hand open)", motion.setAngles, [hand_joint], [0.0], sp)


def _say(tts, text):
    """Speak a line (best-effort)."""
    if not text:
        return
    try:
        _nao_try("ALTextToSpeech.say", tts.say, str(text))
    except Exception:
        pass


def _print_stiffness(motion, joint_names, label):
    """Print stiffness readback (best-effort) for debugging."""
    try:
        vals = _nao_try("ALMotion.getStiffnesses(%s)" % label, motion.getStiffnesses, list(joint_names))
        vals = [float(v) for v in vals]
        if not vals:
            return
        print("%s stiffness: min=%.2f max=%.2f avg=%.2f" % (label, min(vals), max(vals), sum(vals) / len(vals)))
    except Exception:
        pass


def _print_hand(motion, hand_joint, label):
    """Print hand sensor value (best-effort)."""
    try:
        v = _nao_try("ALMotion.getAngles(%s)" % hand_joint, motion.getAngles, [hand_joint], True)[0]
        print("%s %s sensor: %.3f" % (label, hand_joint, float(v)))
    except Exception:
        pass


def _celebration_dance(motion, duration_s, arm_joints, speed_fraction):
    """Slow, low-amplitude dance using only the selected arm joints."""
    names = list(arm_joints)
    if not names:
        return

    try:
        base = _nao_try("ALMotion.getAngles(dance base)", motion.getAngles, names, True)
    except Exception:
        base = [0.0 for _ in names]

    limits = {}
    for j in names:
        try:
            lim = _nao_try("ALMotion.getLimits(%s)" % j, motion.getLimits, j)
            limits[j] = (float(lim[0][0]), float(lim[0][1]))
        except Exception:
            limits[j] = (-3.14, 3.14)

    sp = max(0.05, min(0.35, float(speed_fraction)))
    dt = 0.20
    amp = 0.18  # radians

    start = time.time()
    while (time.time() - start) < float(duration_s):
        t = time.time() - start
        targets = []
        for j, b in zip(names, base):
            v = float(b)
            if "ShoulderRoll" in j:
                v = float(b) + amp * math.sin(2.0 * math.pi * 0.20 * t)
            elif "ElbowRoll" in j:
                v = float(b) + 0.5 * amp * math.sin(2.0 * math.pi * 0.20 * t + 1.2)
            elif "ShoulderPitch" in j:
                v = float(b) + 0.5 * amp * math.sin(2.0 * math.pi * 0.12 * t + 0.5)
            jmin, jmax = limits.get(j, (-3.14, 3.14))
            if v < jmin:
                v = jmin
            if v > jmax:
                v = jmax
            targets.append(float(v))
        try:
            _nao_try("ALMotion.setAngles(dance slow)", motion.setAngles, names, targets, sp)
        except Exception:
            pass
        time.sleep(dt)


def _parse_args():
    """Parse CLI arguments."""
    p = argparse.ArgumentParser(description="Xbox operator for Pepper Touch (no ROS)")
    p.add_argument("--ip", required=True, help="Pepper IP address")
    p.add_argument("--port", type=int, default=PORT, help="NAOqi port (default: 9559)")
    p.add_argument("--dev", default=JOYSTICK_DEV, help="Joystick device (default: /dev/input/js0)")
    p.add_argument(
        "--side",
        choices=["left", "right"],
        default="right",
        help="Which arm/hand to use (default: right)",
    )
    # Convenience aliases (common typing mistakes)
    p.add_argument(
        "--sideright",
        action="store_const",
        const="right",
        dest="side",
        help="Alias for `--side right`",
    )
    p.add_argument(
        "--sideleft",
        action="store_const",
        const="left",
        dest="side",
        help="Alias for `--side left`",
    )
    p.add_argument("--lb-button", type=int, default=4, help="LB button number (default: 4)")
    p.add_argument("--rb-button", type=int, default=5, help="RB button number (default: 5)")
    p.add_argument("--a-button", type=int, default=0, help="A button number (default: 0)")
    p.add_argument("--b-button", type=int, default=1, help="B button number (default: 1)")
    p.add_argument("--lt-axis", type=int, default=2, help="LT axis number (default: 2)")
    p.add_argument("--rt-axis", type=int, default=5, help="RT axis number (default: 5)")
    p.add_argument(
        "--disable-triggers",
        action="store_true",
        help="Disable LT/RT axis handling (useful if triggers are noisy or mapped differently)",
    )
    p.add_argument(
        "--trigger-threshold",
        type=int,
        default=20000,
        help="LT/RT press threshold for axis value (default: 20000; range is usually 0..32767)",
    )
    return p.parse_args()


def main():
    """Entrypoint."""
    args = _parse_args()
    print("Pepper Xbox operator starting...")
    print("Pepper IP: %s" % args.ip)
    print("Joystick device: %s" % args.dev)
    if not os.path.exists(args.dev):
        raise SystemExit("Joystick device not found: %s" % args.dev)

    get_service = _connect_naoqi(args.ip, args.port)
    _disable_autonomy(get_service)
    motion = get_service(MOD_MOTION)
    try:
        tts = get_service(MOD_TTS)
    except Exception:
        tts = None
    try:
        posture = get_service(MOD_POSTURE)
    except Exception:
        posture = None
    _wake_up(motion)

    # Side selection (left vs right).
    if args.side == "right":
        arm_joints = list(RIGHT_ARM_JOINTS)
        hand_joint = RIGHT_HAND
        wrist_joint = RIGHT_WRIST
    else:
        arm_joints = list(LEFT_ARM_JOINTS)
        hand_joint = LEFT_HAND
        wrist_joint = LEFT_WRIST

    # Hold pose while locked (include wrist so the wrist angle never changes).
    hold_joints = list(arm_joints)
    stiffness_debug_joints = list(hold_joints) + [wrist_joint, hand_joint]

    wrist_stop = threading.Event()
    wrist_target = {"angle": 0.0}
    if WRIST_HOLD_ENABLED:
        wrist_target["angle"] = float(_get_wrist_yaw(motion, wrist_joint))
        _start_wrist_hold_thread(motion, wrist_joint, wrist_target, wrist_stop)

    lock_hold_stop = threading.Event()
    lock_hold_thread = None

    def _cleanup():
        """Open and free arm on exit (best-effort)."""
        try:
            wrist_stop.set()
        except Exception:
            pass
        try:
            lock_hold_stop.set()
        except Exception:
            pass
        try:
            _open_hand(motion, hand_joint)
        except Exception:
            pass
        try:
            _set_free_stiffness(motion, arm_joints, hand_joint, wrist_joint)
        except Exception:
            pass

    atexit.register(_cleanup)

    print("Opening joystick: %s" % args.dev)
    print("Mapping:")
    print("  A: LOCK pose + squeeze (hard hold)")
    print("  B: free + open (can take 10–20s; be patient)")
    print("  X: small squish")
    print("  Y: HARD squish (max safe)")
    print("  RT: autoplace right arm onto table + lock")
    print("  RB: slow dance")
    print("  START: quit")
    print("Begin pressing controller buttons now...")
    _set_free_stiffness(motion, arm_joints, hand_joint, wrist_joint)
    _open_hand(motion, hand_joint)

    def _do_lock_squeeze(source):
        nonlocal lock_hold_stop, lock_hold_thread
        print("%s: LOCK + squeeze" % source)
        _wake_up(motion)
        # Hard lock and hold the current pose.
        _set_lock_stiffness(motion, arm_joints, hand_joint, wrist_joint)
        time.sleep(0.05)
        _set_lock_stiffness(motion, arm_joints, hand_joint, wrist_joint)
        _print_stiffness(motion, stiffness_debug_joints, "after LOCK")
        try:
            lock_hold_stop.set()
        except Exception:
            pass
        lock_hold_stop = threading.Event()
        lock_hold_thread = _start_pose_hold_thread(motion, hold_joints, lock_hold_stop)
        if LOCK_SQUEEZE_VALUE > 0.0:
            _squish(motion, hand_joint, LOCK_SQUEEZE_VALUE, LOCK_SQUEEZE_SPEED, LOCK_SQUEEZE_HOLD_S)
            _print_hand(motion, hand_joint, "after LOCK squeeze")

    def _do_free():
        print("FREE + open")
        _wake_up(motion)
        _open_hand(motion, hand_joint)
        _set_free_stiffness(motion, arm_joints, hand_joint, wrist_joint)
        _print_stiffness(motion, stiffness_debug_joints, "after FREE")
        try:
            lock_hold_stop.set()
        except Exception:
            pass

    def _do_small():
        print("small squish")
        _squish(motion, hand_joint, SQUISH_SMALL_CLOSE, SQUISH_SMALL_SPEED, SQUISH_SMALL_HOLD_S)

    def _do_hard():
        print("HARD squish")
        _squish(motion, hand_joint, SQUISH_HARD_CLOSE, SQUISH_HARD_SPEED, SQUISH_HARD_HOLD_S)

    def _do_rt_autoplace():
        print("RT: autoplace right arm to table + lock")
        if args.side != "right":
            print("RT: run with --side right")
            return
        _wake_up(motion)
        try:
            lock_hold_stop.set()
        except Exception:
            pass
        _open_hand(motion, hand_joint)
        _autoplace_right_arm_to_table(motion)
        _do_lock_squeeze("RT")

    def _do_dance():
        print("RB: slow dance (8s)")
        try:
            lock_hold_stop.set()
        except Exception:
            pass
        _celebration_dance(motion, duration_s=8.0, arm_joints=arm_joints, speed_fraction=0.18)
        # Return to a normal upright posture afterwards (best-effort).
        try:
            _open_hand(motion, hand_joint)
        except Exception:
            pass
        try:
            if posture is not None:
                _nao_try("ALRobotPosture.goToPosture(StandInit)", posture.goToPosture, "StandInit", 0.25)
        except Exception:
            pass

    # Axis-trigger debouncing (LT/RT are often axes, not buttons).
    lt_active = False
    rt_active = False
    threshold = int(max(0, min(32767, args.trigger_threshold)))
    release_threshold = int(max(0, threshold // 2))

    try:
        fd = os.open(args.dev, os.O_RDONLY)
    except PermissionError:
        raise SystemExit(
            "Permission denied opening %s. Try: sudo python3 pepper_xbox_operator.py --ip %s"
            % (args.dev, args.ip)
        )
    try:
        while True:
            data = os.read(fd, _JS_EVENT_STRUCT.size)
            if len(data) != _JS_EVENT_STRUCT.size:
                continue
            _t_ms, value, ev_type, number = _JS_EVENT_STRUCT.unpack(data)
            ev_type = ev_type & ~_JS_EVENT_INIT
            if ev_type == _JS_EVENT_AXIS and (not args.disable_triggers):
                v = int(value)
                if int(number) == int(args.lt_axis) and int(args.lt_axis) >= 0:
                    if (not lt_active) and v >= threshold:
                        lt_active = True
                        # LT unused
                    elif lt_active and v <= release_threshold:
                        lt_active = False
                if int(number) == int(args.rt_axis) and int(args.rt_axis) >= 0:
                    if (not rt_active) and v >= threshold:
                        rt_active = True
                        _do_rt_autoplace()
                    elif rt_active and v <= release_threshold:
                        rt_active = False
                continue

            if ev_type != _JS_EVENT_BUTTON:
                continue

            pressed = int(value) == 1
            if not pressed:
                continue

            # Common Xbox button numbers on Linux:
            # 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 7=START (can vary by controller/driver)
            if number == 7:
                break
            if int(number) == int(args.a_button):
                _do_lock_squeeze("A")
                continue
            if int(number) == int(args.b_button):
                print("B")
                _do_free()
                continue
            if number == 2:
                print("X")
                _do_small()
                continue
            if number == 3:
                print("Y")
                _do_hard()
                continue
            if int(number) == int(args.rb_button) and int(args.rb_button) >= 0:
                _do_dance()
                continue
            print("Button pressed (unmapped): number=%d" % int(number))
    finally:
        try:
            os.close(fd)
        except Exception:
            pass
        _cleanup()
        print("Exited.")


if __name__ == "__main__":
    main()
