#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read Pepper joint angles (right arm focused) for Touch experiment tuning."""

import argparse
import math
import os
import re
import sys
from contextlib import redirect_stderr


# =========================
# Tunable config (optional)
# =========================

PORT = 9559

# Joints relevant for Touch tuning (radians). Add/remove if needed.
LEFT_ARM_JOINTS = [
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
]

OPTIONAL_STILLNESS_JOINTS = [
    "HeadYaw",
    "HeadPitch",
]

HAND_JOINTS = ["LHand"]


def _parse_args():
    """Parse CLI arguments."""
    p = argparse.ArgumentParser(description="Read Pepper joint angles for Touch tuning")
    p.add_argument("--ip", required=True, help="Pepper IP address (e.g., 192.168.1.10)")
    p.add_argument("--port", type=int, default=PORT, help="NAOqi port (default: 9559)")
    p.add_argument(
        "--include-head",
        action="store_true",
        help="Also print head angles (useful to confirm stillness)",
    )
    p.add_argument(
        "--include-hand",
        action="store_true",
        help="Also print RHand sensor value (0=open, 1=closed)",
    )
    p.add_argument(
        "--apply-to",
        choices=["none", "node", "test", "both"],
        default="none",
        help="Overwrite DEFAULT_ARM_ANGLES in pepper_touch_node.py and/or pepper_test.py",
    )
    return p.parse_args()


def _connect_naoqi(ip, port):
    """Connect to NAOqi via qi.Session (preferred) or naoqi.ALProxy (fallback)."""
    try:
        with open(os.devnull, "w") as _devnull, redirect_stderr(_devnull):
            import qi  # type: ignore

        # Create a qi.Application to avoid noisy qi.path warnings.
        app = qi.Application(["read_angles", "--qi-url=tcp://%s:%d" % (ip, int(port))])
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


def _dict_block_from_angles(angles_by_name):
    """Render a copy/paste-ready DEFAULT_ARM_ANGLES dict block (radians)."""
    lines = ["DEFAULT_ARM_ANGLES = {"]  # matches our other scripts
    for k in LEFT_ARM_JOINTS:
        lines.append('    "%s": %.6f,' % (k, float(angles_by_name[k])))
    lines.append("}")
    return "\n".join(lines) + "\n"


def _replace_default_arm_angles(path, dict_block):
    """Replace DEFAULT_ARM_ANGLES dict in a Python file (best-effort, in-place)."""
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()

    pat = re.compile(r"DEFAULT_ARM_ANGLES\\s*=\\s*\\{.*?\\}\\s*", re.DOTALL)
    if not pat.search(src):
        raise RuntimeError("DEFAULT_ARM_ANGLES block not found in %s" % path)

    out = pat.sub(dict_block + "\n", src, count=1)
    with open(path, "w", encoding="utf-8") as f:
        f.write(out)


def main():
    """Entrypoint."""
    args = _parse_args()

    get_service = _connect_naoqi(args.ip, args.port)
    motion = get_service("ALMotion")

    names = list(LEFT_ARM_JOINTS)
    if args.include_head:
        names += OPTIONAL_STILLNESS_JOINTS
    if args.include_hand:
        names += HAND_JOINTS

    # True = read from sensors (actual posture)
    angles = motion.getAngles(names, True)

    print("Raw sensor angles (radians; hands are 0..1):")
    for n, a in zip(names, angles):
        print("  %s: %.6f" % (n, float(a)))

    print("\nHuman-friendly (degrees; hands remain 0..1):")
    for n, a in zip(names, angles):
        if n.endswith("Hand"):
            print("  %s: %.3f" % (n, float(a)))
        else:
            print("  %s: %.1f deg" % (n, math.degrees(float(a))))

    # Copy/paste-ready dict for pepper_touch_node.py / pepper_test.py
    arm_only = dict(zip(LEFT_ARM_JOINTS, angles[: len(LEFT_ARM_JOINTS)]))
    dict_block = _dict_block_from_angles(arm_only)
    print("\nCopy/paste into DEFAULT_ARM_ANGLES (radians):")
    print(dict_block.rstrip())

    if args.apply_to != "none":
        base_dir = os.path.dirname(os.path.abspath(__file__))
        node_path = os.path.join(base_dir, "pepper_touch_node.py")
        test_path = os.path.join(base_dir, "pepper_test.py")

        if args.apply_to in ("node", "both"):
            _replace_default_arm_angles(node_path, dict_block)
            print("\nUpdated DEFAULT_ARM_ANGLES in: %s" % node_path)
        if args.apply_to in ("test", "both"):
            _replace_default_arm_angles(test_path, dict_block)
            print("Updated DEFAULT_ARM_ANGLES in: %s" % test_path)
        sys.stdout.flush()


if __name__ == "__main__":
    main()
