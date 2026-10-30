"""
ROB-UY 3303 - Lab 03: Path Following
STARTER CODE - complete every function marked STUDENT TODO.
Run the unit tests with:   python lab_3.py
The GUI (gui_03.py) imports this file and uses your controllers on the robot.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them.

Units: meters, seconds, radians.  v = forward speed (m/s), w = rotational
speed (rad/s, + = counter-clockwise), phi_dot_l / phi_dot_r = wheel speeds (rad/s).
"""
import math
import matplotlib.pyplot as plt
# =====================================================================
# STUDENT TODO - ROBOT CONSTANTS (your Lab 2 values, same as in gui_03.py)
# =====================================================================
WHEEL_RADIUS_M = 0.067/2.0      # wheel radius r (m)
WHEELBASE_M = 0.206        # distance between the wheels = 2L (m)


# =====================================================================
# PART 1 - WHEEL VELOCITIES
# =====================================================================
# STUDENT TODO: get_wheel_velocities(v, w) -> (phi_dot_l, phi_dot_r)
# Use the equations you derived in Step 1.
def get_wheel_velocities(v, w):
    L=WHEELBASE_M/2.0
    phi_dot_r = (v+w*L)/WHEEL_RADIUS_M
    phi_dot_l = (v-w*L)/WHEEL_RADIUS_M
    return phi_dot_l,phi_dot_r

#GEMINI VERSION
# def get_wheel_velocities(v, w):
#     """
#     Converts robot forward (v) and angular (w) velocity to left and right 
#     wheel rotational speeds in rad/s.
#     """
#     L = WHEELBASE_M / 2.0
#     phi_dot_r = (v + w * L) / WHEEL_RADIUS_M
#     phi_dot_l = (v - w * L) / WHEEL_RADIUS_M
#     return phi_dot_l, phi_dot_r


# STUDENT TODO: 5 unit tests, each returns True if it passes.
#   1: v > 0, w = 0     2: v < 0, w = 0     3: v = 0, w > 0
#   4: v = 0, w < 0     5: v = 0, w = 0
    



TOL = 1e-9


def _close(a, b):
    return abs(a - b) < TOL


def wheel_velocity_unit_test_1():
    phi_dot_l,phi_dot_r= get_wheel_velocities(1, 0)
    print(phi_dot_r,phi_dot_l)
    return _close(phi_dot_l,phi_dot_r) and phi_dot_l>0



def wheel_velocity_unit_test_2():
    phi_dot_l,phi_dot_r= get_wheel_velocities(-1, 0)
    print(phi_dot_l,phi_dot_r)
    return _close(-1*phi_dot_l,-1*phi_dot_r) and phi_dot_l<0

def wheel_velocity_unit_test_3():
    phi_dot_l,phi_dot_r=  get_wheel_velocities(0, 1)
    print(phi_dot_l,phi_dot_r)
    return _close(phi_dot_l,-1*phi_dot_r) and phi_dot_r>0

def wheel_velocity_unit_test_4():
    phi_dot_l,phi_dot_r=  get_wheel_velocities(0, -1)
    print(phi_dot_l,phi_dot_r)
    return _close(phi_dot_l,-1*phi_dot_r) and phi_dot_r<0 

def wheel_velocity_unit_test_5():
    phi_dot_l,phi_dot_r=  get_wheel_velocities(0, 0)
    print(phi_dot_l,phi_dot_r)
    return _close(phi_dot_l,0) and _close(phi_dot_r,0)
# STUDENT TODO: run the 5 tests and print how many pass out of 5.
def run_wheel_velocity_unit_tests():
    tests = [
        wheel_velocity_unit_test_1(),
        wheel_velocity_unit_test_2(),
        wheel_velocity_unit_test_3(),
        wheel_velocity_unit_test_4(),
        wheel_velocity_unit_test_5()
    ]

    print(f"{sum(tests)} / 5 Passed")
    for i in range(5):
        print(f"Test {i+1}: {tests[i]}")

    
# =====================================================================
# PROVIDED - ROBOT SIMULATOR (do not change)
# =====================================================================
DT = 0.1                    # s, time step (same as the GUI control loop)
POS_TOL = 0.03              # m, goal reached if closer than this
ANG_TOL = 0.1               # rad, and heading error smaller than this


def wrap_angle(a):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def get_robot_velocities(phi_dot_l, phi_dot_r):
    """Wheel speeds (rad/s) -> robot (v, w)."""
    L = WHEELBASE_M / 2.0
    v = WHEEL_RADIUS_M * (phi_dot_r + phi_dot_l) / 2.0
    w = WHEEL_RADIUS_M * (phi_dot_r - phi_dot_l) / (2.0 * L)
    return v, w


def robot_simulator(X_tm1, phi_dot_l, phi_dot_r, delta_T, n_substeps=10):
    """Moves the robot for delta_T seconds with constant wheel speeds.
    Returns X_t (new pose [x, y, theta]) and the list of poses visited."""
    x, y, th = X_tm1
    h = delta_T / n_substeps
    poses = []
    for _ in range(n_substeps):
        v, w = get_robot_velocities(phi_dot_l, phi_dot_r)
        x += v * h * math.cos(th + w * h / 2.0)
        y += v * h * math.sin(th + w * h / 2.0)
        th = wrap_angle(th + w * h)
        poses.append([x, y, th])
    return [x, y, th], poses


def is_at_goal(X_des, X):
    """True if X is within POS_TOL and ANG_TOL of X_des."""
    return (math.hypot(X_des[0] - X[0], X_des[1] - X[1]) < POS_TOL and
            abs(wrap_angle(X_des[2] - X[2])) < ANG_TOL)


# =====================================================================
# PART 3 - POINT TRACKING
# =====================================================================
# STUDENT TODO: point_tracking_controller(X_des, X) -> (phi_dot_l, phi_dot_r)
#   X_des = desired pose [x, y, theta], X = current pose [x, y, theta].
#   Use the controller of Lecture 05A (rho, alpha, beta, gains, backwards
#   method), then get_wheel_velocities(v, w). Add any helper functions you need.
pass  # TODO: replace this with your implementation




def point_tracking_controller(X_des, X):
    """Pose controller using rho/alpha/beta and optional reverse driving."""
    dx = X_des[0] - X[0]
    dy = X_des[1] - X[1]
    rho = math.hypot(dx, dy)
    heading_error = wrap_angle(X_des[2] - X[2])

    # Once at the target position, only correct heading.
    if rho < 0.02:
        v = 0.0
        w = 1.5 * heading_error
    else:
        alpha = wrap_angle(math.atan2(dy, dx) - X[2])
        backwards = abs(alpha) > math.pi / 2.0
        if backwards:
            # Use the reverse direction to avoid an unnecessary 180-degree turn.
            alpha = wrap_angle(math.atan2(-dy, -dx) - X[2])
        beta = wrap_angle(heading_error - alpha)

        k_rho, k_alpha, k_beta = 1.0, 2.0, -1.0
        v = (-1.0 if backwards else 1.0) * min(k_rho * rho, 0.22)
        w = k_alpha * alpha + k_beta * beta

    # Limit commands so that neither wheel is asked to exceed its speed range.
    w = max(-2.0, min(2.0, w))
    phi_l, phi_r = get_wheel_velocities(v, w)
    max_wheel_speed = 8.0  # rad/s; below the GUI simulator's 10 rad/s maximum
    scale = min(1.0, max_wheel_speed / max(abs(phi_l), abs(phi_r), 1e-9))
    return phi_l * scale, phi_r * scale

#GEMINI VERSION
# def point_tracking_controller(X_des, X, k_rho=1.5, k_alpha=3.0, k_beta=-1.0):
#     """
#     Enhanced point-tracking controller with bidirectional driving,
#     near-goal orientation stabilization, and speed clamping.
#     """
#     x, y, theta = X
#     x_des, y_des, theta_des = X_des
    
#     # Position and global heading errors
#     dx = x_des - x
#     dy = y_des - y
#     rho = math.hypot(dx, dy)
#     delta_theta = wrap_angle(theta_des - theta)

#     # 1. Near-Goal Deadband
#     # When sitting on the target, stop linear motion and purely adjust orientation
#     if rho < 1e-3:
#         v = 0.0
#         w = 1.5 * delta_theta
#         return get_wheel_velocities(v, w)

#     # 2. Forward Bearing and Alpha Calculation
#     target_bearing = math.atan2(dy, dx)
#     alpha = wrap_angle(target_bearing - theta)

#     # 3. Bidirectional Driving (Pick forward or reverse based on shortest turn)
#     direction = 1.0
#     if abs(alpha) > math.pi / 2.0:
#         direction = -1.0
#         alpha = wrap_angle(alpha + math.pi)

#     # 4. Beta Calculation and Control Signals
#     beta = wrap_angle(delta_theta - alpha)

#     v = direction * k_rho * rho
#     w = k_alpha * alpha + k_beta * beta

#     # 5. Velocity Clamping (Prevents wheel saturation during large maneuvers)
#     v = max(min(v, 1.2), -1.2)   # Max linear velocity: +/- 1.2 m/s
#     w = max(min(w, 2.5), -2.5)   # Max angular velocity: +/- 2.5 rad/s

#     return get_wheel_velocities(v, w)


# STUDENT TODO: 6 unit tests (cases in the assignment), each returns
# (passed, visited_poses). Loop: point_tracking_controller -> robot_simulator
# until is_at_goal(X_des, X). Add a time limit so a test can fail.
pass  # TODO: replace this with your implementation


def point_tracker_unit_test_1():
    X= [0,0,0]
    X_des = [2,2,0]
    time_limit = 60
    sim_time = 0.0
    full_path=[X]
    while not is_at_goal(X_des,X):
        phi_dot_l, phi_dot_r = point_tracking_controller(X_des,X)
        X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
        full_path.extend(path)
        sim_time+=DT
        if sim_time>= time_limit:
            return False, full_path
    return True, full_path
   


def point_tracker_unit_test_2():
    X=[0,0,0]
    X_des=[2,-2,0]
    time_limit = 60
    sim_time = 0.0
    full_path=[X]
    while not is_at_goal(X_des,X):
        phi_dot_l, phi_dot_r = point_tracking_controller(X_des,X)
        X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
        full_path.extend(path)
        sim_time+=DT
        if sim_time>= time_limit:
            return False, full_path
    return True, full_path



def point_tracker_unit_test_3():
    X=[0,0,0]
    X_des=[-2,2,0]
    time_limit = 60
    sim_time = 0.0
    full_path=[X]
    while not is_at_goal(X_des,X):
        phi_dot_l, phi_dot_r = point_tracking_controller(X_des,X)
        X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
        full_path.extend(path)
        sim_time+=DT
        if sim_time>= time_limit:
            return False, full_path
    return True, full_path
    

def point_tracker_unit_test_4():
   X=[0,0,0]
   X_des=[-2,-2,0]
   time_limit = 60
   sim_time = 0.0
   full_path=[X]
   while not is_at_goal(X_des,X):
        phi_dot_l, phi_dot_r = point_tracking_controller(X_des,X)
        X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
        full_path.extend(path)
        sim_time+=DT
        if sim_time>= time_limit:
            return False, full_path
   return True, full_path
   


def point_tracker_unit_test_5():
    X=[0,0,0]
    X_des=[2,2,math.pi]
    time_limit = 60
    sim_time = 0.0
    full_path=[X]
    while not is_at_goal(X_des,X):
        phi_dot_l, phi_dot_r = point_tracking_controller(X_des,X)
        X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
        full_path.extend(path)
        sim_time+=DT
        if sim_time>= time_limit:
            return False, full_path
    return True, full_path
     

def point_tracker_unit_test_6():
    X=[1,1,0]
    X_des=[3,3,math.pi]
    time_limit = 60
    sim_time = 0.0
    full_path=[X]
    while not is_at_goal(X_des,X):
        phi_dot_l, phi_dot_r = point_tracking_controller(X_des,X)
        X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
        full_path.extend(path)
        sim_time+=DT
        if sim_time>= time_limit:
            return False, full_path
    return True, full_path
    




# STUDENT TODO: run the 6 tests, print how many pass, plot all paths on one figure.


def run_point_tracker_unit_tests(plot=True):
    tests = [
        point_tracker_unit_test_1(),
        point_tracker_unit_test_2(),
        point_tracker_unit_test_3(),
        point_tracker_unit_test_4(),
        point_tracker_unit_test_5(),
        point_tracker_unit_test_6()
    ]
    
    passed_count = sum(passed for passed, path in tests)
    print(f"{passed_count} / {len(tests)} Passed")
    
    for i, (passed, path) in enumerate(tests):
        print(f"Test {i+1}: {passed}")

    if plot:
        plt.figure(figsize=(8, 8))
        
        for i, (passed, path) in enumerate(tests):
            xs = [pose[0] for pose in path]
            ys = [pose[1] for pose in path]
            
            plt.plot(xs, ys, label=f"Test {i+1}", linewidth=2)
            plt.plot(xs[0], ys[0], 'go', markersize=6)
            plt.plot(xs[-1], ys[-1], 'rx', markersize=8)

        plt.axhline(0, color='black', linewidth=1.5, zorder=1)
        plt.axvline(0, color='black', linewidth=1.5, zorder=1)

        plt.xlim(-4.5, 4.5)
        plt.ylim(-4.5, 4.5)

        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Part 3: Point Tracking Controller Unit Tests")
        plt.axis("equal")  
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc="upper left")
        plt.show()

# =====================================================================
# PART 4 - PATH TRACKING
# =====================================================================
# STUDENT TODO: path_tracking_controller(P, X) -> (phi_dot_l, phi_dot_r)
#   P = path, list of poses [[x, y, theta], ...], X = current pose.
#   Use the method of Lecture 06A (closest point, look-ahead distance Delta,
#   point tracking to that point). Add any helper functions you need.


def path_tracking_controller(P, X):
    """Follow a polyline using the closest path point and a forward lookahead."""
    if not P:
        return 0.0, 0.0
    if len(P) == 1:
        return point_tracking_controller(P[0], X)

    lookahead = 0.20  # meters; increase for smoother turns, decrease for precision

    # Find the closest *point on a segment*, not just the closest waypoint.
    best_distance_sq = float("inf")
    closest_segment = 0
    closest_fraction = 0.0
    for i in range(len(P) - 1):
        ax, ay = P[i][:2]
        bx, by = P[i + 1][:2]
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-12:  # repeated position (e.g. in-place heading change)
            continue
        t = max(0.0, min(1.0, ((X[0] - ax) * dx + (X[1] - ay) * dy) / length_sq))
        px, py = ax + t * dx, ay + t * dy
        d_sq = (X[0] - px) ** 2 + (X[1] - py) ** 2
        if d_sq <= best_distance_sq:
            best_distance_sq, closest_segment, closest_fraction = d_sq, i, t

    # Walk 'lookahead' meters forward along successive segments.
    remaining = lookahead
    for i in range(closest_segment, len(P) - 1):
        a, b = P[i], P[i + 1]
        seg_length = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg_length < 1e-12:
            continue
        t0 = closest_fraction if i == closest_segment else 0.0
        available = (1.0 - t0) * seg_length
        if remaining <= available:
            t = t0 + remaining / seg_length
            theta = wrap_angle(a[2] + t * wrap_angle(b[2] - a[2]))
            target = [a[0] + t * (b[0] - a[0]),
                      a[1] + t * (b[1] - a[1]), theta]
            return point_tracking_controller(target, X)
        remaining -= available

    # Finish at the final waypoint, including its desired orientation.
    return point_tracking_controller(P[-1], X)

#GEMINI VERSION
# def path_tracking_controller(P, X, delta_D=0.5):
#     """
#     Path-tracking controller using a look-ahead target point.
    
#     1. Finds closest pose p_c,t on path P relative to robot pose X[cite: 26].
#     2. Searches forward along path P for a look-ahead pose p_des,t at distance >= delta_D[cite: 24, 26].
#     3. Calls point_tracking_controller(p_des,t, X)[cite: 25, 26].
#     """
#     # 1. Find the index of the closest pose on path P (p_ct)[cite: 26]
#     closest_idx = 0
#     min_dist = float('inf')
    
#     for i, pose in enumerate(P):
#         dist = math.hypot(pose[0] - X[0], pose[1] - X[1])
#         if dist < min_dist:
#             min_dist = dist
#             closest_idx = i

#     p_ct = P[closest_idx]

#     # 2. Look ahead for target pose (p_dest) at least delta_D away[cite: 24, 26]
#     p_dest = P[-1]  # Fallback: end of path if remaining distance < delta_D[cite: 24]
    
#     for i in range(closest_idx, len(P)):
#         dist = math.hypot(P[i][0] - p_ct[0], P[i][1] - p_ct[1])
#         if dist >= delta_D:
#             p_dest = P[i]
#             break

#     # 3. Compute control outputs towards p_dest[cite: 25, 26]
#     return point_tracking_controller(p_dest, X)


# STUDENT TODO: 3 unit tests (cases in the assignment), each returns
# (passed, visited_poses). Stop when is_at_goal(P[-1], X).
POS_TOL_1 = 0.02            # m, goal reached if closer than this
ANG_TOL_1 = 0.3           # rad, and heading error smaller than this
def is_at_goal_1(X_des, X):
    """True if X is within POS_TOL and ANG_TOL of X_des."""
    return (math.hypot(X_des[0] - X[0], X_des[1] - X[1]) < POS_TOL_1 and abs(wrap_angle(X_des[2] - X[2])) < ANG_TOL_1)


def path_tracker_unit_test_1():
    X=[0,0,0]
    P=[[0,0.1,0],[2,0.1,0]]
    time_limit=60
    sim_time=0.0
    full_path=[X]
    while not is_at_goal_1(P[-1],X):
            phi_dot_l, phi_dot_r = path_tracking_controller(P,X)
            X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
            full_path.extend(path)
            sim_time+=DT
            if sim_time>= time_limit:
                return False, full_path

    return True, full_path




def path_tracker_unit_test_2():
    X=[0,0,0]
    P=[[0,0,0],[1.0,0,0],[1.0,0,math.pi/2],[1.0,1.0,math.pi/2]]
    time_limit=60
    sim_time=0.0
    full_path=[X]
    while not is_at_goal_1(P[-1],X):
            phi_dot_l, phi_dot_r = path_tracking_controller(P,X)
            X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
            full_path.extend(path)
            sim_time+=DT
            if sim_time>= time_limit:
                return False, full_path
    return True, full_path


def path_tracker_unit_test_3():
    X=[0,0,0]
    P=[[0,0,math.pi/4],[2.0,2.0,math.pi/4]]
    time_limit=60
    sim_time=0.0
    full_path=[X]
    while not is_at_goal_1(P[-1],X):
            phi_dot_l, phi_dot_r = path_tracking_controller(P,X)
            X, path = robot_simulator(X,phi_dot_l,phi_dot_r,DT)
            full_path.extend(path)
            sim_time+=DT
            if sim_time>= time_limit:
                return False, full_path
    return True, full_path



# STUDENT TODO: run the 3 tests, print how many pass, plot each path as a subfigure.
def run_path_tracker_unit_tests(plot=True):
    tests = [
        path_tracker_unit_test_1(),
        path_tracker_unit_test_2(),
        path_tracker_unit_test_3()
    ]
        
    passed_count = sum(passed for passed, path in tests)
    print(f"{passed_count} / {len(tests)} Passed")
    
    for i, (passed, path) in enumerate(tests):
        print(f"Test {i+1}: {passed}")

    if plot:
        fig, axes = plt.subplots(1, len(tests), figsize=(15, 5))
        
        for i, (ax, (passed, path)) in enumerate(zip(axes, tests)):
            xs = [pose[0] for pose in path]
            ys = [pose[1] for pose in path]
            
            ax.plot(xs, ys, label=f"Path", color="blue", linewidth=2)
            ax.plot(xs[0], ys[0], 'go', markersize=6, label="Start")
            ax.plot(xs[-1], ys[-1], 'rx', markersize=8, label="End")

            ax.axhline(0, color='black', linewidth=1.5, zorder=1)
            ax.axvline(0, color='black', linewidth=1.5, zorder=1)

            ax.set_xlim(-4.5, 4.5)
            ax.set_ylim(-4.5, 4.5)

            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.set_title(f"Test {i+1} (Passed: {passed})")
            ax.set_aspect("equal")  
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend(loc="upper left")

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    print("=== Part 1: wheel velocities ===")
    run_wheel_velocity_unit_tests()
    print("\n=== Part 3: point tracking ===")
    run_point_tracker_unit_tests()
    try:
        run_extra_point_tests()
    except NameError:
        pass
    print("\n=== Part 4: path tracking ===")
    run_path_tracker_unit_tests()
