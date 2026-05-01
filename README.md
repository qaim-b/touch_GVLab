TLDR (RIGHT arm)
- Xbox controller operator: `python3 pepper_xbox_operator.py --ip 192.168.0.100`

Buttons / triggers (defaults, can be changed with `--a-button N`, `--rt-axis N`, etc):
- `A`: lock current RIGHT arm pose + squeeze once
- `B`: free + open hand
- `X`: small squish
- `Y`: hard squish (max safe)
- `RT` (trigger): autoplace RIGHT arm onto the table (elbow bends first) + lock
- `RB`: slow dance (safe, low speed)
- `START`: quit

- Manual hotkeys (keyboard): `python3 pepper_manual_hotkey.py --ip 192.168.0.100`
  - `5` lock (+ squeeze), `0` free, `o` open, `q` quit



A TLDR:

1. pepper_xbox_operator.py — main operator script: Xbox controller → Pepper hold/squeeze/dance + optional 2. speech.pepper_manual_hotkey.py — keyboard hotkeys (no controller): free/lock arm + squish.
3. pepper_touch_node.py — ROS node: listens to a Stroop topic and triggers squish + CSV logging.
4. pepper_test.py — no-ROS verification: move to default pose, run small+large squish, log test result.
5. read.py — utility to read current joint angles from Pepper and print/apply them to defaults.





---------------------------------------------------------------
# Touch (GVLab) — Pepper Left-Arm “Squish” Experiment

Pepper rests its **left arm/hand** on the participant’s forearm and performs brief “squish” motions (light finger close) triggered by the Stroop game end-of-round topic.

Pepper is configured to be **silent and still**: no speech, no blinking/LED animations, no autonomous movement. Only the squish varies by round.

## What you need before starting

- Pepper is powered on and standing safely at the table.
- TP-Link portable router is powered on.
- Laptop is connected to the **same Wi‑Fi network** as Pepper (via the TP-Link router).
- `naoqi_driver` is already running on Pepper (as you noted in setup).

## How to find Pepper’s IP

Press Pepper’s chest button once; Pepper reads the IP address aloud.

## Install dependencies (ROS Noetic + Python 3)

Run **at most these 3 commands**:

1. `pip3 install --user qi`
2. `pip3 install --user naoqi`
3. `pip3 install --user rospkg`

Notes:
- If you only have one NAOqi SDK installed, that’s fine: the scripts try `import qi` first and fall back to `import naoqi`.
- ROS Python packages (`rospy`, `std_msgs`) come from your ROS Noetic installation.

## Run the verification test first (no ROS)

This moves the left arm to the default pose, then runs a **small** squish and a **large** squish:

`python3 pepper_test.py --ip 192.168.0.100`

Expected output:
- Prints the actual joint angles read back.
- Prints the `LHand` angle during each hold.
- Creates/updates `squish_log_test.csv` in this folder.

## Main experiment (controller-only, no ROS)

Pepper does not need to “know about the game”. The operator controls touch timing directly using keyboard or Xbox controller.

## ROS trigger mode (optional)

If you want the Stroop game to trigger squishes automatically via a ROS topic, run `pepper_touch_node.py`.

Example (String topic; any message triggers, and you can include `small` / `large` in the text to override):

`source /opt/ros/noetic/setup.bash`

`roscore`

`python3 pepper_touch_node.py _ip:=192.168.0.100 _trigger_topic:=/stroop/answer _msg_type:=String`

Example (Bool topic; only `True` triggers):

`python3 pepper_touch_node.py _ip:=192.168.0.100 _trigger_topic:=/stroop/round_end _msg_type:=Bool`

Useful params:
- `_csv_path:=squish_log.csv`
- `_squish_sequence:="[small, large]"` (repeats when payload doesn’t specify)
- `_init_pose:=true` (move to `DEFAULT_ARM_ANGLES` on startup)

## CSV log format

File: `squish_log.csv` (created on startup, appended each round)

Columns:
- `timestamp_iso`
- `round_number`
- `squish_type`
- `hand_close_value`
- `hold_duration_s`
- `success`

## Stop safely

Press `CTRL+C` in the terminal running the script.

On exit the scripts attempt to:
- Open the hand fully
- Move the left arm to `REST_ARM_ANGLES`
- Re-enable autonomous life (restore previous state when possible)

## Manual mode (participant positions the arm)

If you want the participant to position Pepper’s left arm by hand, then “lock” it with a hotkey:

`python3 pepper_manual_hotkey.py --ip 192.168.0.100`

Hotkeys:
- `5`: lock (stiffness=1.0) + squeeze (safe `LHand` close)
- `0`: free (stiffness=0.0) + open hand
- `o`: open hand only
- `q`: quit (cleanup)

## Xbox controller operator (optional, no ROS)

If an Xbox controller is connected to the Pepper-control laptop, you can operate without the keyboard:

`python3 pepper_xbox_operator.py --ip 192.168.0.100`

Operator guide (what each input does):
- `A`: HOLD in-air pose + squeeze once (can take **10–20s** to fully “settle” — be patient)
- `B`: free stiffness + open hand (can take **10–20s** to fully “settle” — be patient)
- `X`: small squish
- `Y`: HARD squish (max safe)
- `LB`: introduction speech (start-of-session)
- `LT`: between-rounds speech (+ free+open; trigger axis press)
- `RT`: last-round-ended speech (+ free+open; trigger axis press)
- `RB`: 5-second celebration dance
- `START`: quit

Optional:
- Use right arm/hand for debugging: add `--side right` (or `--sideright`)

## Troubleshooting (common)

1. **Cannot connect to Pepper / timeout**
   - Confirm laptop is on the TP-Link Wi‑Fi and the IP is correct (`--ip 192.168.1.X`).

2. **ImportError: qi / naoqi**
   - Install one of them (`pip3 install --user qi` or `pip3 install --user naoqi`).

3. **ROS topic does not trigger**
   - Confirm the Stroop node is publishing `/stroop/answer` and that `STROOP_MSG_TYPE` matches (`String` vs `Bool`).
