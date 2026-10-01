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

# =====================================================================
# STUDENT TODO - ROBOT CONSTANTS (your Lab 2 values, same as in gui_03.py)
# =====================================================================
WHEEL_RADIUS_M = None     # wheel radius r (m)
WHEELBASE_M = None        # distance between the wheels = 2L (m)


# =====================================================================
# PART 1 - WHEEL VELOCITIES
# =====================================================================
# STUDENT TODO: get_wheel_velocities(v, w) -> (phi_dot_l, phi_dot_r)
# Use the equations you derived in Step 1.
def get_wheel_velocities(v, w):
    pass  # TODO: replace this with your implementation


# STUDENT TODO: 5 unit tests, each returns True if it passes.
#   1: v > 0, w = 0     2: v < 0, w = 0     3: v = 0, w > 0
#   4: v = 0, w < 0     5: v = 0, w = 0
TOL = 1e-9


def _close(a, b):
    return abs(a - b) < TOL


def wheel_velocity_unit_test_1():
    pass  # TODO: replace this with your implementation


def wheel_velocity_unit_test_2():
    pass  # TODO: replace this with your implementation


def wheel_velocity_unit_test_3():
    pass  # TODO: replace this with your implementation


def wheel_velocity_unit_test_4():
    pass  # TODO: replace this with your implementation


def wheel_velocity_unit_test_5():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 5 tests and print how many pass out of 5.
def run_wheel_velocity_unit_tests():
    pass  # TODO: replace this with your implementation


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
    pass  # TODO: replace this with your implementation


# STUDENT TODO: 6 unit tests (cases in the assignment), each returns
# (passed, visited_poses). Loop: point_tracking_controller -> robot_simulator
# until is_at_goal(X_des, X). Add a time limit so a test can fail.
pass  # TODO: replace this with your implementation


def point_tracker_unit_test_1():
    pass  # TODO: replace this with your implementation


def point_tracker_unit_test_2():
    pass  # TODO: replace this with your implementation


def point_tracker_unit_test_3():
    pass  # TODO: replace this with your implementation


def point_tracker_unit_test_4():
    pass  # TODO: replace this with your implementation


def point_tracker_unit_test_5():
    pass  # TODO: replace this with your implementation


def point_tracker_unit_test_6():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 6 tests, print how many pass, plot all paths on one figure.
def run_point_tracker_unit_tests(plot=True):
    pass  # TODO: replace this with your implementation


pass  # TODO: replace this with your implementation


# =====================================================================
# PART 4 - PATH TRACKING
# =====================================================================
# STUDENT TODO: path_tracking_controller(P, X) -> (phi_dot_l, phi_dot_r)
#   P = path, list of poses [[x, y, theta], ...], X = current pose.
#   Use the method of Lecture 06A (closest point, look-ahead distance Delta,
#   point tracking to that point). Add any helper functions you need.
pass  # TODO: replace this with your implementation


def path_tracking_controller(P, X):
    pass  # TODO: replace this with your implementation


# STUDENT TODO: 3 unit tests (cases in the assignment), each returns
# (passed, visited_poses). Stop when is_at_goal(P[-1], X).
pass  # TODO: replace this with your implementation


def path_tracker_unit_test_1():
    pass  # TODO: replace this with your implementation


def path_tracker_unit_test_2():
    pass  # TODO: replace this with your implementation


def path_tracker_unit_test_3():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 3 tests, print how many pass, plot each path as a subfigure.
def run_path_tracker_unit_tests(plot=True):
    pass  # TODO: replace this with your implementation


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
