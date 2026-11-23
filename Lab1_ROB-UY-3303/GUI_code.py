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

# --- LIDAR VIEW (no live localization) ---
ROBOT_RADIUS_CM = 10.0
LIDAR_OFFSET_ANGLE = 0.0

loop_after_id = None
timeout_counter = 0

# --- RAW TELEMETRY (encoders/gyro are sensor readings only, never used to decide when to stop) ---
last_raw_enc_a = 0
last_raw_enc_b = 0
last_raw_gyro = 0.0
last_raw_angle = 0.0
last_lidar = None
data_updated = False

# --- STATE MACHINE ---
nav_state = "IDLE"
raw_move_start_time = 0.0
raw_move_duration = 0.0
raw_move_pwm_a = 0
raw_move_pwm_b = 0

# --- CSV RECORDING ---
recording_active = False
recording_start_time = 0.0
csv_file_handle = None
csv_writer = None


def draw_map():
    canvas_map.delete("all")
    SCALE = 4.5

    # Robot marker fixed at the canvas center, oriented by the gyro heading (no live position tracking)
    rx, ry = 325, 200
    rad = ROBOT_RADIUS_CM * SCALE
    heading_rad = math.radians(last_raw_angle)
    canvas_map.create_oval(rx-rad, ry-rad, rx+rad, ry+rad, outline="#00FF00", width=2)

    hx = rx - rad * math.cos(heading_rad)
    hy = ry - rad * math.sin(heading_rad)
    canvas_map.create_line(rx, ry, hx, hy, fill="#00FF00", width=2)

    if last_lidar:
        for i in range(0, 360, 5):
            dist_mm = last_lidar[i // 5]
            if dist_mm > 0 and dist_mm < 3000:
                dist_cm = dist_mm / 10.0
                ray_ang = heading_rad + math.radians(i + LIDAR_OFFSET_ANGLE)
                lx = rx - (dist_cm * SCALE) * math.cos(ray_ang)
                ly = ry - (dist_cm * SCALE) * math.sin(ray_ang)
                canvas_map.create_line(rx, ry, lx, ly, fill="#550000")
                canvas_map.create_oval(lx-2, ly-2, lx+2, ly+2, fill="red", outline="")


def start_raw_move(pwm_a, pwm_b, duration_s):
    global nav_state, raw_move_start_time, raw_move_duration, raw_move_pwm_a, raw_move_pwm_b
    raw_move_pwm_a = pwm_a
    raw_move_pwm_b = pwm_b
    raw_move_duration = duration_s
    raw_move_start_time = time.time()
    nav_state = "RAW_MOVE"
    lbl_nav_status.config(text=f"State: RAW_MOVE (pwm {pwm_a},{pwm_b} for {duration_s:.2f}s)", fg="#FEE75C")


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
    global nav_state
    nav_state = "IDLE"
    lbl_nav_status.config(text="State: IDLE (Emergency Stop)", fg="#FF0000")
    try: sock.sendto(f"M,{var_motor_a.get()},{var_motor_b.get()},0,0".encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception: pass


def toggle_recording():
    global recording_active, recording_start_time, csv_file_handle, csv_writer
    if not recording_active:
        current_dir = os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) else os.path.dirname(os.path.abspath(__file__))
        file_name = time.strftime("raw_log_%Y%m%d_%H%M%S.csv")
        full_path = os.path.join(current_dir, file_name)
        csv_file_handle = open(full_path, 'w', newline='')
        csv_writer = csv.writer(csv_file_handle)
        csv_writer.writerow(["time_s", "enc_a", "enc_b", "gyro_rate_dps", "angle_deg"])
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
    csv_writer.writerow([f"{elapsed:.3f}", last_raw_enc_a, last_raw_enc_b, f"{last_raw_gyro:.2f}", f"{last_raw_angle:.2f}"])


def decide_state_and_pwm(en_a, en_b):
    """State machine: IDLE (manual sliders) or RAW_MOVE (hold a fixed PWM pair for a fixed duration)."""
    global nav_state

    pwm_a, pwm_b = 0, 0

    if nav_state == "RAW_MOVE":
        if time.time() - raw_move_start_time >= raw_move_duration:
            nav_state = "IDLE"
            lbl_nav_status.config(text="State: IDLE (Completed)", fg="#00FF00")
        else:
            pwm_a, pwm_b = raw_move_pwm_a, raw_move_pwm_b

    if nav_state == "IDLE":
        vel_raw, rot_raw = slider_velocity.get(), slider_steering.get()
        if abs(rot_raw) >= DEAD_ZONE: pwm_a, pwm_b = rot_raw, -rot_raw
        elif abs(vel_raw) >= DEAD_ZONE: pwm_a, pwm_b = vel_raw, vel_raw

    return pwm_a, pwm_b


def send_command(message):
    """Sends the current tick's command string to the robot over UDP."""
    try:
        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception:
        pass


def receive_telemetry():
    """Drains any pending ACK packets, updating the raw telemetry globals. Returns whether one was received."""
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
                    last_raw_enc_a = float(parts[1])
                    last_raw_enc_b = float(parts[2])
                    last_raw_gyro = float(parts[3])
                    last_raw_angle = float(parts[4])
                    data_updated = True

                if len(parts) == 77:
                    last_lidar = [int(p) for p in parts[5:77]]
    except BlockingIOError:
        pass

    return ack_received


def update_telemetry_labels():
    """Refreshes the on-screen encoder/gyro labels from the latest telemetry."""
    if not data_updated:
        return
    lbl_enc.config(text=f"Enc A: {last_raw_enc_a}  |  Enc B: {last_raw_enc_b}")
    lbl_gyro.config(text=f"Gyro Z: {last_raw_gyro:.2f} °/s  |  Abs Angle: {last_raw_angle:.2f}°")


def update_connection_watchdog(ack_received):
    """Tracks consecutive missed ACKs and flips the connection indicator after too many in a row."""
    global timeout_counter
    if ack_received:
        timeout_counter = 0
        update_connection(True)
    else:
        timeout_counter += 1
        if timeout_counter > 20: update_connection(False)


def update_connection(connected):
    if connected:
        canvas_indicator.itemconfig(indicator, fill="#00FF00")
        lbl_connection.config(text="CONNECTED", fg="#00FF00")
    else:
        canvas_indicator.itemconfig(indicator, fill="#FF0000")
        lbl_connection.config(text="DISCONNECTED", fg="#FF0000")


def reset_steering(event): slider_steering.set(0)


def control_loop():
    global loop_after_id
    try:
        en_a, en_b = var_motor_a.get(), var_motor_b.get()

        pwm_a, pwm_b = decide_state_and_pwm(en_a, en_b)
        message = f"M,{en_a},{en_b},{int(pwm_a)},{int(pwm_b)}"

        send_command(message)
        ack_received = receive_telemetry()
        update_telemetry_labels()
        record_sample_if_active()
        draw_map()
        update_connection_watchdog(ack_received)

    except Exception as critical_err:
        print(f"\n[!!!] CRITICAL ERROR IN CONTROL LOOP: {critical_err}[!!!]")

    loop_after_id = root.after(100, control_loop)


# --- GRAPHICAL INTERFACE ---
root = tk.Tk()
root.title("Robot Console (Raw)")
root.geometry("1000x700")
root.configure(bg="#2C2F33")

telemetry_style = {"bg": "#2C2F33", "fg": "#00FF00", "font": ("Consolas", 10, "bold")}

frame_main = tk.Frame(root, bg="#2C2F33")
frame_main.pack(fill=tk.BOTH, expand=True, pady=10)

frame_left = tk.Frame(frame_main, bg="#2C2F33")
frame_left.pack(side=tk.LEFT, fill=tk.Y, padx=20)

frame_right = tk.Frame(frame_main, bg="#2C2F33")
frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=20)

# --- LEFT COLUMN ---
frame_status = tk.Frame(frame_left, bg="#2C2F33")
frame_status.pack(pady=(10, 5))
canvas_indicator = tk.Canvas(frame_status, width=30, height=30, bg="#2C2F33", highlightthickness=0)
indicator = canvas_indicator.create_oval(5, 5, 25, 25, fill="#FF0000")
canvas_indicator.pack(side=tk.LEFT)
lbl_connection = tk.Label(frame_status, text="DISCONNECTED", bg="#2C2F33", font=("Arial", 12, "bold"), fg="#FF0000")
lbl_connection.pack(side=tk.LEFT, padx=10)

frame_telemetry = tk.Frame(frame_left, bg="#2C2F33")
frame_telemetry.pack(pady=0)
lbl_enc = tk.Label(frame_telemetry, text="Enc A: 0  |  Enc B: 0.00", **telemetry_style)
lbl_enc.pack()
lbl_gyro = tk.Label(frame_telemetry, text="Gyro Z: 0.00 °/s  |  Abs Angle: 0.00°", **telemetry_style)
lbl_gyro.pack()

lbl_nav_status = tk.Label(frame_left, text="State: IDLE", bg="#2C2F33", fg="#00FF00", font=("Arial", 11, "bold"))
lbl_nav_status.pack(pady=(5, 10))

frame_moti = tk.Frame(frame_left, bg="#2C2F33")
frame_moti.pack(pady=0)
var_motor_a = tk.IntVar(value=1)
var_motor_b = tk.IntVar(value=1)
tk.Checkbutton(frame_moti, text="Motor A", variable=var_motor_a, bg="#2C2F33", fg="white", selectcolor="#23272A").pack(side=tk.LEFT, padx=20)
tk.Checkbutton(frame_moti, text="Motor B", variable=var_motor_b, bg="#2C2F33", fg="white", selectcolor="#23272A").pack(side=tk.LEFT, padx=20)

slider_velocity = tk.Scale(frame_left, from_=255, to=-255, orient=tk.VERTICAL, length=120, width=30, bg="#23272A", fg="white", highlightthickness=0, troughcolor="#7289DA")
slider_velocity.set(0)
slider_velocity.pack()
slider_steering = tk.Scale(frame_left, from_=-255, to=255, orient=tk.HORIZONTAL, length=300, width=30, bg="#23272A", fg="white", highlightthickness=0, troughcolor="#99AAB5")
slider_steering.set(0)
slider_steering.bind("<ButtonRelease-1>", reset_steering)
slider_steering.pack()

tk.Button(frame_left, highlightthickness=0, text="STOP", command=stop_motors, bg="#E74C3C", fg="white", font=("Arial", 9, "bold")).pack(pady=10)

frame_raw = tk.LabelFrame(frame_left, text="Raw Timed Move (open loop, no correction)", bg="#2C2F33", fg="white", font=("Arial", 10, "bold"))
frame_raw.pack(pady=10, fill=tk.X)
tk.Label(frame_raw, text="PWM A:", bg="#2C2F33", fg="white").grid(row=0, column=0, padx=5, pady=5, sticky="e")
entry_pwm_a = tk.Entry(frame_raw, width=6); entry_pwm_a.grid(row=0, column=1, padx=2); entry_pwm_a.insert(0, "150")
tk.Label(frame_raw, text="PWM B:", bg="#2C2F33", fg="white").grid(row=1, column=0, padx=5, pady=5, sticky="e")
entry_pwm_b = tk.Entry(frame_raw, width=6); entry_pwm_b.grid(row=1, column=1, padx=2); entry_pwm_b.insert(0, "150")
tk.Label(frame_raw, text="Duration (s):", bg="#2C2F33", fg="white").grid(row=2, column=0, padx=5, pady=5, sticky="e")
entry_duration = tk.Entry(frame_raw, width=6); entry_duration.grid(row=2, column=1, padx=2); entry_duration.insert(0, "1.0")
tk.Button(frame_raw, highlightthickness=0, text="GO", command=start_raw_move_ui, bg="#7289DA", fg="white", font=("Arial", 9, "bold")).grid(row=0, column=2, rowspan=3, padx=10, pady=5)
tk.Label(frame_raw, text="Same sign PWM A/B = straight, opposite sign = rotate in place", bg="#2C2F33", fg="#99AAB5", font=("Arial", 8)).grid(row=3, column=0, columnspan=3, padx=5, pady=(0, 5))

frame_record = tk.LabelFrame(frame_left, text="Data Recording", bg="#2C2F33", fg="white", font=("Arial", 10, "bold"))
frame_record.pack(pady=10, fill=tk.X)
btn_record = tk.Button(frame_record, highlightthickness=0, text="START RECORDING", command=toggle_recording, bg="#43B581", fg="white", font=("Arial", 9, "bold"))
btn_record.pack(pady=5)
lbl_record_status = tk.Label(frame_record, text="Not recording", bg="#2C2F33", fg="white")
lbl_record_status.pack(pady=(0, 5))

# --- RIGHT COLUMN (MAP) ---
canvas_map = tk.Canvas(frame_right, width=650, height=500, bg="#111111", highlightthickness=2, highlightbackground="#7289DA")
canvas_map.pack(pady=10)


def on_closing():
    if loop_after_id is not None:
        root.after_cancel(loop_after_id)
    if recording_active and csv_file_handle:
        csv_file_handle.close()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_closing)
loop_after_id = root.after(100, control_loop)
root.mainloop()
