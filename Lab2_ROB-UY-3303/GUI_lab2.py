import tkinter as tk
import socket
import time
import csv
import os
import sys
import math

# --- NETWORK CONFIGURATION ---
UDP_IP = "192.168.4.1"
UDP_PORT = 4010
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

DEAD_ZONE = 110

# --- LIDAR VIEW ---
ROBOT_RADIUS_CM = 10.0
LIDAR_OFFSET_ANGLE = 0.0

loop_after_id = None
timeout_counter = 0

# --- RAW TELEMETRY ---
last_raw_enc_a = 0
last_raw_enc_b = 0
last_raw_gyro = 0.0
last_raw_angle = 0.0
last_lidar = None
data_updated = False

# --- STATE MACHINE ---
# States: "IDLE", "RAW_MOVE", "SEQUENCE"
nav_state = "IDLE"
raw_move_pwm_a = 0
raw_move_pwm_b = 0

# Timed moves end on their own root.after() timer rather than on the 100 ms
# control tick, so a 0.4 s turn lasts 0.4 s instead of 3-5 ticks' worth.
step_after_id = None

# --- SEQUENCE (automated square drive) ---
# A sequence is a list of (pwm_a, pwm_b, duration_s) tuples executed back-to-back.
sequence_steps = []
sequence_step_index = 0
SEQUENCE_PAUSE_S = 0.3   # stop between moves so each one starts from rest

# --- CSV RECORDING ---
recording_active = False
recording_start_time = 0.0
csv_file_handle = None
csv_writer = None

# =====================================================================
# ROBOT PHYSICAL CONSTANTS
# =====================================================================
# You will need to measure these values for YOUR robot before running
# the odometry functions below.
#
#   TICKS_PER_REV_LEFT / TICKS_PER_REV_RIGHT :
#                   How many encoder ticks (rising edges on CH1) does each
#                   sensor produce for one full wheel revolution?
#                   Measure each wheel SEPARATELY: lift the robot, mark a
#                   reference point on the tire, drive that wheel for a few
#                   seconds while filming it, then count the wheel turns in
#                   the video and divide the encoder delta by that count.
#                   Use 10-20 turns so that a miscount costs you little.
#                   Do not assume the two wheels come out identical — that
#                   is why the left and right functions below are separate.
#
#   WHEEL_RADIUS_M : The effective radius of the drive wheels in meters.
#                   Measure from the axle center to the outer edge of the
#                   tire (convert cm to m by dividing by 100), then check it
#                   by driving a measured straight line and confirming the
#                   odometry distance matches the tape measure.
#
#   WHEELBASE_M    : The distance between the two wheel contact patches
#                   in meters. This is the full axle width (left contact
#                   point to right contact point). Often called 2L in
#                   the lecture notes where L is the half-wheelbase.
#                   Measure with the robot on a flat surface.
#
# Leave these as placeholders until you have made the physical measurements.
# =====================================================================
TICKS_PER_REV_LEFT = None   # TODO: your measured ticks per revolution, LEFT wheel (encoder A)
TICKS_PER_REV_RIGHT = None  # TODO: your measured ticks per revolution, RIGHT wheel (encoder B)
WHEEL_RADIUS_M = None       # TODO: replace with your measured value (float, meters)
WHEELBASE_M = None          # TODO: replace with your measured value (float, meters)

# --- ODOMETRY STATE ---
# robot_state holds [x, y, theta] in meters and radians (global frame).
# x grows to the right, y grows upward, theta=0 faces right (+x direction).
# Positive theta is counter-clockwise (right-hand rule about +z axis).
robot_state = [0.0, 0.0, 0.0]      # [x_m, y_m, theta_rad]

# We need to remember the encoder counts from the PREVIOUS tick so we can
# compute the delta (change) this tick. Initialized to None so we can
# detect the very first telemetry packet and skip the first delta.
prev_enc_a = None   # left wheel (encoder A) count at last tick
prev_enc_b = None   # right wheel (encoder B) count at last tick

# Path history: list of (x_m, y_m, theta_rad) snapshots, one per tick
# where new telemetry arrived. Used to draw the odometry trail on the canvas.
odometry_path = []


# =====================================================================
# STUDENT TODO: get_wheel_rotation_left
# =====================================================================
# Purpose: Convert a raw encoder tick count (delta, not cumulative) for
#          the LEFT wheel into the angle the wheel has rotated, in radians.
#
# Background:
#   The encoder fires one tick per "slot" in the encoder disc as the wheel
#   turns. After exactly one full wheel revolution the encoder has fired
#   TICKS_PER_REV_LEFT ticks. A partial revolution of (delta_ticks) ticks
#   therefore corresponds to a fraction of a full rotation:
#
#       fraction = delta_ticks / TICKS_PER_REV_LEFT
#
#   One full revolution = 2*pi radians, so:
#
#       rotation_rad = fraction * 2 * pi
#
#   Positive delta_ticks means the wheel moved forward (robot moves forward).
#   Negative delta_ticks means the wheel moved backward.
#
# Parameters:
#   enc_counts  (int or float) — the DELTA encoder count for this tick
#                                (current cumulative count minus previous
#                                 cumulative count). May be negative.
#
# Returns:
#   rotation_rad (float) — signed wheel rotation in radians.
#                          Positive = forward, negative = backward.
#
# Hint: Use the global constant TICKS_PER_REV_LEFT defined above.
# =====================================================================
def get_wheel_rotation_left(enc_counts):
    pass  # replace this with your implementation


# =====================================================================
# STUDENT TODO: get_wheel_rotation_right
# =====================================================================
# Same as get_wheel_rotation_left but for the RIGHT wheel (encoder B).
# The math is identical, but use TICKS_PER_REV_RIGHT — each wheel is
# calibrated on its own and the two counts may not match.
#
# Parameters:
#   enc_counts  (int or float) — the DELTA encoder count for the right wheel.
#
# Returns:
#   rotation_rad (float) — signed wheel rotation in radians.
# =====================================================================
def get_wheel_rotation_right(enc_counts):
    pass  # replace this with your implementation


# =====================================================================
# STUDENT TODO: get_wheel_distance_left
# =====================================================================
# Purpose: Convert a wheel rotation angle (radians) into the arc length
#          the LEFT wheel has traveled along the ground, in meters.
#
# Background:
#   If a wheel of radius r rotates by angle phi (radians), the contact
#   patch travels a distance equal to the arc length:
#
#       distance = r * phi
#
#   This is just the definition of arc length. The sign is preserved:
#   positive phi (forward rotation) gives positive distance.
#
# Parameters:
#   rotation_rad (float) — wheel rotation angle in radians (from
#                          get_wheel_rotation_left).
#
# Returns:
#   distance_m (float) — signed arc length traveled in meters.
#
# Hint: Use the global constant WHEEL_RADIUS_M defined above.
# =====================================================================
def get_wheel_distance_left(rotation_rad):
    pass  # replace this with your implementation


# =====================================================================
# STUDENT TODO: get_wheel_distance_right
# =====================================================================
# Same as get_wheel_distance_left but for the RIGHT wheel.
#
# Parameters:
#   rotation_rad (float) — wheel rotation angle in radians (from
#                          get_wheel_rotation_right).
#
# Returns:
#   distance_m (float) — signed arc length traveled in meters.
# =====================================================================
def get_wheel_distance_right(rotation_rad):
    pass  # replace this with your implementation


# =====================================================================
# STUDENT TODO: get_state_change
# =====================================================================
# Purpose: Given the arc distances each wheel has traveled this tick,
#          compute the CHANGE in robot pose [delta_x, delta_y, delta_theta]
#          in the GLOBAL frame.
#
# Background — Differential Drive Kinematics:
#   For a differential-drive robot (two independently driven wheels),
#   the motion equations for one small timestep are:
#
#   Let:
#     ds_r = arc distance traveled by the RIGHT wheel (meters, signed)
#     ds_l = arc distance traveled by the LEFT wheel  (meters, signed)
#     L    = HALF the wheelbase (= WHEELBASE_M / 2)
#     theta = current heading angle (radians, from robot_state[2])
#
#   Step 1 — Average forward displacement:
#     delta_s = (ds_r + ds_l) / 2
#
#   Step 2 — Change in heading:
#     delta_theta = (ds_r - ds_l) / (2 * L)
#     (positive = counter-clockwise turn, matching our right-hand convention)
#
#   Step 3 — Change in global position:
#     Use the midpoint heading (theta + delta_theta/2) to project delta_s:
#       delta_x = delta_s * cos(theta + delta_theta / 2)
#       delta_y = delta_s * sin(theta + delta_theta / 2)
#
#   These equations come from Lecture 3A (Motion Modeling).
#
# Parameters:
#   dist_right (float) — arc distance of right wheel this tick (meters)
#   dist_left  (float) — arc distance of left wheel this tick (meters)
#
# Returns:
#   [delta_x, delta_y, delta_theta]
#     delta_x     (float) — change in x position (meters, global frame)
#     delta_y     (float) — change in y position (meters, global frame)
#     delta_theta (float) — change in heading (radians, positive = CCW)
#
# Hints:
#   - Access the current heading from robot_state[2].
#   - Use math.cos() and math.sin() (already imported).
#   - Use WHEELBASE_M (the global constant) to compute L = WHEELBASE_M / 2.
# =====================================================================
def get_state_change(dist_right, dist_left):
    pass  # replace this with your implementation


# =====================================================================
# STUDENT TODO: predict_robot_state
# =====================================================================
# Purpose: Given the previous robot pose and the raw encoder counts for
#          both wheels this tick, compute and return the new robot pose.
#
# This function ties all four functions above together into one pipeline:
#
#   Step 1: Compute the delta encoder counts for each wheel.
#             delta_enc_left  = enc_left  - last_enc_left
#             delta_enc_right = enc_right - last_enc_right
#           (enc_left and enc_right are the CURRENT cumulative counts;
#            last_enc_left / last_enc_right are the counts from the
#            previous tick — these are the prev_enc_a / prev_enc_b globals)
#
#   Step 2: Convert delta encoder counts to wheel rotations (radians).
#             rot_left  = get_wheel_rotation_left(delta_enc_left)
#             rot_right = get_wheel_rotation_right(delta_enc_right)
#
#   Step 3: Convert rotations to arc distances (meters).
#             dist_left  = get_wheel_distance_left(rot_left)
#             dist_right = get_wheel_distance_right(rot_right)
#
#   Step 4: Compute the change in pose.
#             [dx, dy, dtheta] = get_state_change(dist_right, dist_left)
#
#   Step 5: Add the change to the previous state to get the new state.
#             new_x     = last_state[0] + dx
#             new_y     = last_state[1] + dy
#             new_theta = last_state[2] + dtheta
#
#   Step 6: Return [new_x, new_y, new_theta].
#
# Parameters:
#   last_state   (list)  — [x, y, theta] from the previous tick
#   enc_right    (float) — current cumulative encoder count, right wheel (enc_b)
#   enc_left     (float) — current cumulative encoder count, left wheel  (enc_a)
#
# Returns:
#   [new_x, new_y, new_theta]  — updated robot pose
#
# Note: The caller (update_odometry) stores prev_enc_a / prev_enc_b after
#       calling this function, so you do NOT need to update those globals here.
# =====================================================================
def predict_robot_state(last_state, enc_right, enc_left):
    pass  # replace this with your implementation


def update_odometry():
    """Called once per control loop tick when new telemetry has arrived.
    Updates robot_state and appends to odometry_path."""
    global robot_state, prev_enc_a, prev_enc_b, odometry_path

    # Skip the very first packet — we have no "previous" counts to delta against.
    if prev_enc_a is None:
        prev_enc_a = last_raw_enc_a
        prev_enc_b = last_raw_enc_b
        return

    # Skip if physical constants are not yet filled in.
    if (TICKS_PER_REV_LEFT is None or TICKS_PER_REV_RIGHT is None
            or WHEEL_RADIUS_M is None or WHEELBASE_M is None):
        return

    new_state = predict_robot_state(robot_state, last_raw_enc_b, last_raw_enc_a)

    if new_state is not None:
        robot_state = new_state
        odometry_path.append(tuple(robot_state))

    prev_enc_a = last_raw_enc_a
    prev_enc_b = last_raw_enc_b


def reset_odometry():
    """Resets the odometry state and path history to the origin."""
    global robot_state, prev_enc_a, prev_enc_b, odometry_path
    robot_state = [0.0, 0.0, 0.0]
    prev_enc_a = None
    prev_enc_b = None
    odometry_path = []
    lbl_odom.config(text="Odometry reset. x=0.00 m  y=0.00 m  θ=0.00°")


def draw_map():
    """Draws the odometry trajectory and current robot pose on the canvas."""
    canvas_map.delete("all")

    # Canvas center = origin of the odometry frame
    cx, cy = 325, 250
    SCALE = 200  # pixels per meter

    # Grid lines every 0.5 m
    for i in range(-4, 5):
        offset = i * SCALE // 2
        canvas_map.create_line(cx + offset, 0, cx + offset, 500, fill="#1a1a2e", width=1)
        canvas_map.create_line(0, cy + offset, 650, cy + offset, fill="#1a1a2e", width=1)

    # Axis labels (1 m marks)
    for i in range(-3, 4):
        if i == 0:
            continue
        px = cx + i * SCALE
        py = cy - i * SCALE
        canvas_map.create_text(px, cy + 12, text=f"{i}m", fill="#444466", font=("Consolas", 7))
        canvas_map.create_text(cx - 20, py, text=f"{i}m", fill="#444466", font=("Consolas", 7))

    # Odometry path trail
    if len(odometry_path) >= 2:
        for i in range(1, len(odometry_path)):
            x0, y0, _ = odometry_path[i - 1]
            x1, y1, _ = odometry_path[i]
            px0 = cx + x0 * SCALE
            py0 = cy - y0 * SCALE
            px1 = cx + x1 * SCALE
            py1 = cy - y1 * SCALE
            canvas_map.create_line(px0, py0, px1, py1, fill="#7289DA", width=2)

    # Pose markers every 10 steps
    for i, (px_m, py_m, th) in enumerate(odometry_path):
        if i % 10 != 0:
            continue
        px = cx + px_m * SCALE
        py = cy - py_m * SCALE
        canvas_map.create_oval(px - 3, py - 3, px + 3, py + 3, fill="#99AAB5", outline="")
        hx = px + 8 * math.cos(th)
        hy = py - 8 * math.sin(th)
        canvas_map.create_line(px, py, hx, hy, fill="#99AAB5", width=1)

    # Current robot position — green circle with heading arrow
    rx = cx + robot_state[0] * SCALE
    ry = cy - robot_state[1] * SCALE
    rad_px = ROBOT_RADIUS_CM * (SCALE / 100.0)
    theta = robot_state[2]
    canvas_map.create_oval(rx - rad_px, ry - rad_px, rx + rad_px, ry + rad_px,
                           outline="#00FF00", width=2)
    hx = rx + rad_px * math.cos(theta)
    hy = ry - rad_px * math.sin(theta)
    canvas_map.create_line(rx, ry, hx, hy, fill="#00FF00", width=2)


# --- SQUARE SEQUENCE ---

def build_square_sequence(side_m, pwm_straight, pwm_turn,
                          straight_duration_s, turn_duration_s):
    """Returns a list of (pwm_a, pwm_b, duration_s) steps that drive a square.

    The sequence alternates four straight segments with four 90° left turns.
    All durations are open-loop — tune pwm and duration values using the
    Raw Timed Move panel first, then enter them in the GUI controls.

    Parameters:
        side_m            -- target side length in meters (informational only)
        pwm_straight      -- PWM value for both wheels when going straight
        pwm_turn          -- PWM magnitude for turning (one wheel fwd, one back)
        straight_duration_s -- seconds to run straight for one side
        turn_duration_s   -- seconds to spin for a 90° left turn
    """
    steps = []
    for _ in range(4):
        steps.append((pwm_straight, pwm_straight, straight_duration_s))   # forward 1 side
        steps.append((0, 0, SEQUENCE_PAUSE_S))                            # let the robot stop coasting
        steps.append((-pwm_turn, pwm_turn, turn_duration_s))              # 90° CCW turn (left back, right fwd)
        steps.append((0, 0, SEQUENCE_PAUSE_S))
    return steps


def start_square_sequence():
    """Reads the square parameters from the UI and launches the sequence."""
    global sequence_steps, nav_state
    try:
        pwm_s = int(entry_sq_pwm_straight.get())
        pwm_t = int(entry_sq_pwm_turn.get())
        dur_s = float(entry_sq_dur_straight.get())
        dur_t = float(entry_sq_dur_turn.get())
    except ValueError:
        return
    cancel_step_timer()
    sequence_steps = build_square_sequence(1.0, pwm_s, pwm_t, dur_s, dur_t)
    nav_state = "SEQUENCE"
    run_sequence_step(0)


def run_sequence_step(index):
    """Starts step `index` immediately and schedules the next step for exactly
    when this one should end."""
    global sequence_step_index, step_after_id, nav_state
    step_after_id = None
    sequence_step_index = index
    if index >= len(sequence_steps):
        nav_state = "IDLE"
        send_motor_command(0, 0)
        lbl_nav_status.config(text="State: IDLE (Sequence done)", fg="#00FF00")
        return
    pwm_a, pwm_b, duration_s = sequence_steps[index]
    send_motor_command(pwm_a, pwm_b)
    lbl_nav_status.config(text=f"State: SEQUENCE (step {index + 1}/{len(sequence_steps)})", fg="#FEE75C")
    step_after_id = root.after(int(round(duration_s * 1000)), run_sequence_step, index + 1)


def start_raw_move(pwm_a, pwm_b, duration_s):
    global nav_state, raw_move_pwm_a, raw_move_pwm_b, step_after_id
    cancel_step_timer()
    raw_move_pwm_a = pwm_a
    raw_move_pwm_b = pwm_b
    nav_state = "RAW_MOVE"
    send_motor_command(pwm_a, pwm_b)
    lbl_nav_status.config(text=f"State: RAW_MOVE (pwm {pwm_a},{pwm_b} for {duration_s:.2f}s)", fg="#FEE75C")
    step_after_id = root.after(int(round(duration_s * 1000)), end_raw_move)


def end_raw_move():
    global nav_state, step_after_id
    step_after_id = None
    nav_state = "IDLE"
    send_motor_command(0, 0)
    lbl_nav_status.config(text="State: IDLE (Completed)", fg="#00FF00")


def cancel_step_timer():
    global step_after_id
    if step_after_id is not None:
        root.after_cancel(step_after_id)
        step_after_id = None


def start_raw_move_ui():
    try:
        pwm_a = int(entry_pwm_a.get())
        pwm_b = int(entry_pwm_b.get())
        duration_s = float(entry_duration.get())
        if duration_s > 0:
            start_raw_move(pwm_a, pwm_b, duration_s)
    except ValueError:
        pass


def stop_motors():
    global nav_state, sequence_steps
    cancel_step_timer()
    nav_state = "IDLE"
    sequence_steps = []
    # Zero the sliders too, otherwise IDLE mode resumes driving on the next tick
    slider_velocity.set(0)
    slider_steering.set(0)
    lbl_nav_status.config(text="State: IDLE (Emergency Stop)", fg="#FF0000")
    send_motor_command(0, 0)


def toggle_recording():
    global recording_active, recording_start_time, csv_file_handle, csv_writer
    if not recording_active:
        current_dir = (os.path.dirname(sys.executable) if getattr(sys, 'frozen', False)
                       else os.path.dirname(os.path.abspath(__file__)))
        file_name = time.strftime("odom_log_%Y%m%d_%H%M%S.csv")
        full_path = os.path.join(current_dir, file_name)
        csv_file_handle = open(full_path, 'w', newline='')
        csv_writer = csv.writer(csv_file_handle)
        csv_writer.writerow(["time_s", "enc_a", "enc_b", "gyro_rate_dps", "angle_deg",
                             "odom_x_m", "odom_y_m", "odom_theta_rad"])
        recording_start_time = time.time()
        recording_active = True
        btn_record.config(text="STOP RECORDING")
        lbl_record_status.config(text=f"Recording: {file_name}", fg="#FF0000")
    else:
        recording_active = False
        csv_file_handle.close()
        btn_record.config(text="START RECORDING")
        lbl_record_status.config(text="Not recording", fg="white")


def record_sample_if_active():
    if not recording_active or not data_updated:
        return
    elapsed = time.time() - recording_start_time
    csv_writer.writerow([
        f"{elapsed:.3f}",
        last_raw_enc_a, last_raw_enc_b,
        f"{last_raw_gyro:.2f}", f"{last_raw_angle:.2f}",
        f"{robot_state[0]:.4f}", f"{robot_state[1]:.4f}", f"{robot_state[2]:.4f}"
    ])


def decide_state_and_pwm(en_a, en_b):
    """State machine: IDLE (manual sliders), RAW_MOVE, or SEQUENCE.
    Timed moves advance on their own timers; this just resends the current
    command every tick."""
    pwm_a, pwm_b = 0, 0

    if nav_state == "RAW_MOVE":
        pwm_a, pwm_b = raw_move_pwm_a, raw_move_pwm_b

    elif nav_state == "SEQUENCE":
        pwm_a, pwm_b, _ = sequence_steps[sequence_step_index]

    if nav_state == "IDLE":
        vel_raw, rot_raw = slider_velocity.get(), slider_steering.get()
        if abs(rot_raw) >= DEAD_ZONE:
            pwm_a, pwm_b = rot_raw, -rot_raw
        elif abs(vel_raw) >= DEAD_ZONE:
            pwm_a, pwm_b = vel_raw, vel_raw

    return pwm_a, pwm_b


def send_command(message):
    try:
        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception:
        pass


def send_motor_command(pwm_a, pwm_b):
    """Sends a motor command right away instead of waiting for the next control tick."""
    send_command(f"M,{var_motor_a.get()},{var_motor_b.get()},{int(pwm_a)},{int(pwm_b)}")


def receive_telemetry():
    global last_raw_enc_a, last_raw_enc_b, last_raw_gyro, last_raw_angle, last_lidar, data_updated

    ack_received = False
    data_updated = False

    try:
        while True:
            data, addr = sock.recvfrom(2048)
            received_message = data.decode('utf-8')
            parts = received_message.split(',')

            if parts[0] == "ACK":
                ack_received = True
                if len(parts) >= 5:
                    last_raw_enc_a = int(parts[1])
                    last_raw_enc_b = int(parts[2])
                    last_raw_gyro = float(parts[3])
                    last_raw_angle = float(parts[4])
                    data_updated = True

                if len(parts) == 77:
                    last_lidar = [int(p) for p in parts[5:77]]
    except (BlockingIOError, ConnectionResetError):
        # ConnectionResetError: Windows raises this on UDP recv when the robot is unreachable
        pass

    return ack_received


def update_telemetry_labels():
    if not data_updated:
        return
    lbl_enc.config(text=f"Enc A: {last_raw_enc_a}  |  Enc B: {last_raw_enc_b}")
    lbl_gyro.config(text=f"Gyro Z: {last_raw_gyro:.2f} °/s  |  Abs Angle: {last_raw_angle:.2f}°")
    lbl_odom.config(
        text=f"Odometry  x={robot_state[0]:.3f} m  y={robot_state[1]:.3f} m  "
             f"θ={math.degrees(robot_state[2]):.1f}°"
    )


def update_connection_watchdog(ack_received):
    global timeout_counter
    if ack_received:
        timeout_counter = 0
        update_connection(True)
    else:
        timeout_counter += 1
        if timeout_counter > 20:
            update_connection(False)


def update_connection(connected):
    if connected:
        canvas_indicator.itemconfig(indicator, fill="#00FF00")
        lbl_connection.config(text="CONNECTED", fg="#00FF00")
    else:
        canvas_indicator.itemconfig(indicator, fill="#FF0000")
        lbl_connection.config(text="DISCONNECTED", fg="#FF0000")


def reset_steering(event):
    slider_steering.set(0)


def control_loop():
    global loop_after_id
    try:
        en_a, en_b = var_motor_a.get(), var_motor_b.get()

        pwm_a, pwm_b = decide_state_and_pwm(en_a, en_b)
        message = f"M,{en_a},{en_b},{int(pwm_a)},{int(pwm_b)}"

        send_command(message)
        ack_received = receive_telemetry()

        if data_updated:
            update_odometry()

        update_telemetry_labels()
        record_sample_if_active()
        draw_map()
        update_connection_watchdog(ack_received)

    except Exception as critical_err:
        print(f"\n[!!!] CRITICAL ERROR IN CONTROL LOOP: {critical_err} [!!!]")

    loop_after_id = root.after(100, control_loop)


# =============================================================================
# GRAPHICAL INTERFACE
# =============================================================================
root = tk.Tk()
root.title("Lab 2 — Motion Modeling (Differential Drive Odometry)")
root.geometry("1100x750")
root.configure(bg="#2C2F33")

telemetry_style = {"bg": "#2C2F33", "fg": "#00FF00", "font": ("Consolas", 10, "bold")}

frame_main = tk.Frame(root, bg="#2C2F33")
frame_main.pack(fill=tk.BOTH, expand=True, pady=10)

frame_left = tk.Frame(frame_main, bg="#2C2F33")
frame_left.pack(side=tk.LEFT, fill=tk.Y, padx=10)

frame_right = tk.Frame(frame_main, bg="#2C2F33")
frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10)

# --- LEFT COLUMN: Connection ---
frame_status = tk.Frame(frame_left, bg="#2C2F33")
frame_status.pack(pady=(10, 5))
canvas_indicator = tk.Canvas(frame_status, width=30, height=30, bg="#2C2F33", highlightthickness=0)
indicator = canvas_indicator.create_oval(5, 5, 25, 25, fill="#FF0000")
canvas_indicator.pack(side=tk.LEFT)
lbl_connection = tk.Label(frame_status, text="DISCONNECTED", bg="#2C2F33",
                          font=("Arial", 12, "bold"), fg="#FF0000")
lbl_connection.pack(side=tk.LEFT, padx=10)

# --- Telemetry ---
frame_telemetry = tk.Frame(frame_left, bg="#2C2F33")
frame_telemetry.pack(pady=0)
lbl_enc = tk.Label(frame_telemetry, text="Enc A: 0  |  Enc B: 0", **telemetry_style)
lbl_enc.pack()
lbl_gyro = tk.Label(frame_telemetry, text="Gyro Z: 0.00 °/s  |  Abs Angle: 0.00°", **telemetry_style)
lbl_gyro.pack()
lbl_odom = tk.Label(frame_telemetry,
                    text="Odometry  x=0.000 m  y=0.000 m  θ=0.0°",
                    bg="#2C2F33", fg="#FEE75C", font=("Consolas", 10, "bold"))
lbl_odom.pack()

lbl_nav_status = tk.Label(frame_left, text="State: IDLE", bg="#2C2F33",
                           fg="#00FF00", font=("Arial", 11, "bold"))
lbl_nav_status.pack(pady=(5, 5))

# --- Motor enable checkboxes ---
frame_moti = tk.Frame(frame_left, bg="#2C2F33")
frame_moti.pack(pady=0)
var_motor_a = tk.IntVar(value=1)
var_motor_b = tk.IntVar(value=1)
tk.Checkbutton(frame_moti, text="Motor A (Left)", variable=var_motor_a,
               bg="#2C2F33", fg="white", selectcolor="#23272A").pack(side=tk.LEFT, padx=10)
tk.Checkbutton(frame_moti, text="Motor B (Right)", variable=var_motor_b,
               bg="#2C2F33", fg="white", selectcolor="#23272A").pack(side=tk.LEFT, padx=10)

# --- Manual drive sliders ---
slider_velocity = tk.Scale(frame_left, from_=255, to=-255, orient=tk.VERTICAL,
                           length=100, width=25, bg="#23272A", fg="white",
                           highlightthickness=0, troughcolor="#7289DA")
slider_velocity.set(0)
slider_velocity.pack()
slider_steering = tk.Scale(frame_left, from_=-255, to=255, orient=tk.HORIZONTAL,
                           length=280, width=25, bg="#23272A", fg="white",
                           highlightthickness=0, troughcolor="#99AAB5")
slider_steering.set(0)
slider_steering.bind("<ButtonRelease-1>", reset_steering)
slider_steering.pack()

tk.Button(frame_left, highlightthickness=0, text="STOP", command=stop_motors,
          bg="#E74C3C", fg="white", font=("Arial", 9, "bold")).pack(pady=5)

# --- Reset odometry button ---
tk.Button(frame_left, highlightthickness=0, text="RESET ODOMETRY", command=reset_odometry,
          bg="#E67E22", fg="white", font=("Arial", 9, "bold")).pack(pady=2)

# --- Raw Timed Move ---
frame_raw = tk.LabelFrame(frame_left, text="Raw Timed Move (open loop)",
                          bg="#2C2F33", fg="white", font=("Arial", 9, "bold"))
frame_raw.pack(pady=5, fill=tk.X)
tk.Label(frame_raw, text="PWM A:", bg="#2C2F33", fg="white").grid(row=0, column=0, padx=5, pady=3, sticky="e")
entry_pwm_a = tk.Entry(frame_raw, width=6)
entry_pwm_a.grid(row=0, column=1, padx=2)
entry_pwm_a.insert(0, "150")
tk.Label(frame_raw, text="PWM B:", bg="#2C2F33", fg="white").grid(row=1, column=0, padx=5, pady=3, sticky="e")
entry_pwm_b = tk.Entry(frame_raw, width=6)
entry_pwm_b.grid(row=1, column=1, padx=2)
entry_pwm_b.insert(0, "150")
tk.Label(frame_raw, text="Duration (s):", bg="#2C2F33", fg="white").grid(row=2, column=0, padx=5, pady=3, sticky="e")
entry_duration = tk.Entry(frame_raw, width=6)
entry_duration.grid(row=2, column=1, padx=2)
entry_duration.insert(0, "1.0")
tk.Button(frame_raw, highlightthickness=0, text="GO", command=start_raw_move_ui,
          bg="#7289DA", fg="white", font=("Arial", 9, "bold")).grid(row=0, column=2, rowspan=3, padx=8, pady=3)
tk.Label(frame_raw,
         text="Same-sign PWM A/B = straight, opposite = rotate",
         bg="#2C2F33", fg="#99AAB5", font=("Arial", 7)).grid(row=3, column=0, columnspan=3, padx=5, pady=(0, 3))

# --- Square Sequence ---
frame_sq = tk.LabelFrame(frame_left, text="1 m × 1 m Square Sequence (open loop)",
                         bg="#2C2F33", fg="white", font=("Arial", 9, "bold"))
frame_sq.pack(pady=5, fill=tk.X)
tk.Label(frame_sq, text="PWM straight:", bg="#2C2F33", fg="white").grid(row=0, column=0, padx=5, pady=3, sticky="e")
entry_sq_pwm_straight = tk.Entry(frame_sq, width=6)
entry_sq_pwm_straight.grid(row=0, column=1, padx=2)
entry_sq_pwm_straight.insert(0, "150")
tk.Label(frame_sq, text="Duration (s):", bg="#2C2F33", fg="white").grid(row=1, column=0, padx=5, pady=3, sticky="e")
entry_sq_dur_straight = tk.Entry(frame_sq, width=6)
entry_sq_dur_straight.grid(row=1, column=1, padx=2)
entry_sq_dur_straight.insert(0, "2.0")
tk.Label(frame_sq, text="PWM turn:", bg="#2C2F33", fg="white").grid(row=2, column=0, padx=5, pady=3, sticky="e")
entry_sq_pwm_turn = tk.Entry(frame_sq, width=6)
entry_sq_pwm_turn.grid(row=2, column=1, padx=2)
entry_sq_pwm_turn.insert(0, "150")
tk.Label(frame_sq, text="Turn dur (s):", bg="#2C2F33", fg="white").grid(row=3, column=0, padx=5, pady=3, sticky="e")
entry_sq_dur_turn = tk.Entry(frame_sq, width=6)
entry_sq_dur_turn.grid(row=3, column=1, padx=2)
entry_sq_dur_turn.insert(0, "0.9")
tk.Button(frame_sq, highlightthickness=0, text="RUN SQUARE", command=start_square_sequence,
          bg="#43B581", fg="white", font=("Arial", 9, "bold")).grid(row=0, column=2, rowspan=4, padx=8, pady=3)
tk.Label(frame_sq,
         text="Tune PWM and durations with Raw Timed Move first",
         bg="#2C2F33", fg="#99AAB5", font=("Arial", 7)).grid(row=4, column=0, columnspan=3, padx=5, pady=(0, 3))

# --- Data Recording ---
frame_record = tk.LabelFrame(frame_left, text="Data Recording",
                             bg="#2C2F33", fg="white", font=("Arial", 9, "bold"))
frame_record.pack(pady=5, fill=tk.X)
btn_record = tk.Button(frame_record, highlightthickness=0, text="START RECORDING",
                       command=toggle_recording, bg="#43B581", fg="white", font=("Arial", 9, "bold"))
btn_record.pack(pady=4)
lbl_record_status = tk.Label(frame_record, text="Not recording", bg="#2C2F33", fg="white")
lbl_record_status.pack(pady=(0, 4))

# --- RIGHT COLUMN: Odometry Map ---
tk.Label(frame_right, text="Odometry Trajectory (top-down, global frame)",
         bg="#2C2F33", fg="#99AAB5", font=("Arial", 10)).pack()
canvas_map = tk.Canvas(frame_right, width=650, height=500, bg="#111111",
                       highlightthickness=2, highlightbackground="#7289DA")
canvas_map.pack(pady=5)
tk.Label(frame_right,
         text="Blue trail = odometry path  |  Green = current pose  |  Grid lines every 0.5 m",
         bg="#2C2F33", fg="#444466", font=("Arial", 8)).pack()


def on_closing():
    if loop_after_id is not None:
        root.after_cancel(loop_after_id)
    cancel_step_timer()
    if recording_active and csv_file_handle:
        csv_file_handle.close()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_closing)
loop_after_id = root.after(100, control_loop)
root.mainloop()
