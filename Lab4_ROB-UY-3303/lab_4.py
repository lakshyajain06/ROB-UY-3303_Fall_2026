"""
ROB-UY 3303 - Lab 04: Tree Search Motion Planning (A*)
STARTER CODE - complete every function marked STUDENT TODO.
Run the unit tests with:   python lab_4.py
The GUI (gui_4.py) imports this file and uses your planner and trackers on
the robot. map_4.py (same folder) holds the maps.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them.

Units: meters, seconds, radians.
Grid: 200 x 200 cells of 1 cm (see map_4.py). A cell is a list [i, j] of
two ints: i = column (along x), j = row (along y). OCCUPANCY_GRID[i][j] is
1 if the CENTRE of the robot cannot be in cell [i, j].
"""
import math
import time

import numpy as np

import map_4

# =====================================================================
# PART 0 - STUDENT TODO: COPY YOUR LAB 3 CODE (lab_3.py)
# =====================================================================
# Copy your robot constants and these three functions from lab_3.py
# (same names, inputs and outputs). Copy any helper function they use too.
WHEEL_RADIUS_M = None     # wheel radius r (m)
WHEELBASE_M = None        # distance between the wheels = 2L (m)


def get_wheel_velocities(v, w):
    pass  # TODO: copy from lab_3.py


def point_tracking_controller(X_des, X):
    pass  # TODO: copy from lab_3.py


def path_tracking_controller(P, X):
    pass  # TODO: copy from lab_3.py


# =====================================================================
# PROVIDED - ROBOT SIMULATOR (same as Lab 3, do not change)
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
# PROVIDED - THE GRID MAP (do not change)
# =====================================================================
GRID_N = map_4.GRID_N                   # 200 cells along x and along y
CELL_SIZE_M = map_4.CELL_SIZE_M         # 0.01 m
ROBOT_SPEED_M_S = 0.12                  # m/s, robot speed on normal floor (k = 1)

OCCUPANCY_GRID = None       # 200 x 200, 1 = robot centre cannot be in the cell
TRAVERSABILITY_GRID = None  # 200 x 200, k in (0, 1]: speed allowed = k * ROBOT_SPEED_M_S
COLLISION_GRID = None       # 200 x 200, 1 = the robot body touches an obstacle or a wall


def load_map(name):
    """Loads a map of map_4.MAPS: the obstacles grown by the robot radius
    and a safety margin (configuration space, Lecture 08A), and the
    traversability."""
    global COLLISION_GRID
    load_grid(map_4.build_c_space(name), map_4.build_traversability_grid(name))
    COLLISION_GRID = map_4.grow_obstacles(map_4.build_obstacle_grid(name), map_4.ROBOT_RADIUS_M)


def load_grid(occupancy, traversability=None):
    """Loads any 200 x 200 occupancy grid (1 = occupied) directly, e.g. for
    the unit tests. traversability = None means k = 1 everywhere."""
    global OCCUPANCY_GRID, TRAVERSABILITY_GRID, COLLISION_GRID
    OCCUPANCY_GRID = np.asarray(occupancy, dtype=np.uint8)
    COLLISION_GRID = OCCUPANCY_GRID
    if traversability is None:
        traversability = np.ones((GRID_N, GRID_N))
    TRAVERSABILITY_GRID = np.asarray(traversability, dtype=float)


def empty_grid():
    """200 x 200 grid of zeros, to build test maps: grid[i][j] = 1 ..."""
    return np.zeros((GRID_N, GRID_N), dtype=np.uint8)


def in_bounds(X):
    """True if cell X = [i, j] is on the grid."""
    return 0 <= X[0] < GRID_N and 0 <= X[1] < GRID_N


def is_occupied(X):
    """True if cell X = [i, j] is in collision."""
    return OCCUPANCY_GRID[X[0]][X[1]] == 1


def traversability(X):
    """Traversability k of cell X = [i, j]."""
    return float(TRAVERSABILITY_GRID[X[0]][X[1]])


def new_cost_array():
    """200 x 200 array full of infinity (one value per cell)."""
    return np.full((GRID_N, GRID_N), math.inf)


def meters_to_cell(x, y):
    """Position (m) -> cell [i, j] that contains it."""
    return [int(math.floor((x - map_4.GRID_ORIGIN_M[0]) / CELL_SIZE_M)),
            int(math.floor((y - map_4.GRID_ORIGIN_M[1]) / CELL_SIZE_M))]


def cell_to_meters(X):
    """Cell [i, j] -> position (m) [x, y] of its centre."""
    return [map_4.GRID_ORIGIN_M[0] + (X[0] + 0.5) * CELL_SIZE_M,
            map_4.GRID_ORIGIN_M[1] + (X[1] + 0.5) * CELL_SIZE_M]


def robot_in_collision(X):
    """True if the robot at pose X = [x, y, theta] (m) touches an obstacle
    or a wall of the map loaded with load_map (no safety margin)."""
    i, j = meters_to_cell(X[0], X[1])
    return not in_bounds([i, j]) or COLLISION_GRID[i][j] == 1


def check_path(P, X_0, X_d):
    """Checks a planned path of cells. Returns (True, "ok") or (False, reason)."""
    if not P:
        return False, "no path returned"
    if list(P[0]) != list(X_0) or list(P[-1]) != list(X_d):
        return False, "path must start at X_0 and end at X_d"
    for k, X in enumerate(P):
        if not in_bounds(X):
            return False, f"cell {k} {X} is off the grid"
        if is_occupied(X) and k > 0:
            return False, f"cell {k} {X} is in collision"
        if k > 0 and abs(X[0] - P[k - 1][0]) + abs(X[1] - P[k - 1][1]) != 1:
            return False, f"cells {k - 1} and {k} are not neighbors"
    return True, "ok"


def path_time_s(P):
    """Time (s) to drive along the path of cells P at the speed allowed in
    each cell entered (k * ROBOT_SPEED_M_S)."""
    return sum(CELL_SIZE_M / (ROBOT_SPEED_M_S * traversability(X)) for X in P[1:])


# =====================================================================
# PART 2 - THE A* PLANNER (Lecture 09A and 09B)
# =====================================================================
# DO NOT USE ANY GEN AI TOOLS OR EXAMPLE A* CODE FOR THIS PART (Step 6 only).
#
# Two cost functions are used in this lab (cost_mode):
#   "distance": shortest path. The cost of a step to a neighbor cell is the
#               distance travelled (m). Part 2.
#   "time":     minimum time. The cost of a step is the time (s) needed to
#               drive into the neighbor cell at the speed allowed there,
#               k * ROBOT_SPEED_M_S. Part 3.
# For now, write everything for "distance". You add "time" in Part 3.

# STUDENT TODO - Step 1: the Node class.
# Member variables:
#   X        - [i, j], the cell of the node
#   parent   - the parent Node (None for the root)
#   g_value  - g(n): path cost from the root to this node
#   h_value  - h(n): estimated path cost from this node to the goal
#   f_value  - f(n) = g(n) + h(n)
class Node:
    def __init__(self, X, parent, g_value, h_value):
        pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 1: the cost of one step and the heuristic.
# get_step_cost(X_from, X_to, cost_mode) -> cost of moving from cell X_from
#   to the neighbor cell X_to (m for "distance", s for "time").
# get_heuristic(X, X_d, cost_mode) -> h(n) for a node at cell X and the goal
#   cell X_d. It must be ADMISSIBLE (never overestimates the cost to the goal,
#   Lecture 09B): use the straight line distance from X to X_d (m).
def get_step_cost(X_from, X_to, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


def get_heuristic(X, X_d, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 2: get_neighbors(n, X_d, best_g, cost_mode)
# Returns a list of up to 4 NEW nodes, children of node n: the cells one
# step in +x, -x, +y and -y from n.X. Set the parent, g, h and f of each.
# Do not create a neighbor if it is:
#   - out of bounds:  off the grid (use in_bounds)
#   - in collision:   occupied (use is_occupied)
#   - already added:  best_g is a 200 x 200 array (new_cost_array()) with
#                     the lowest g found so far for each cell. Only create
#                     the neighbor if its g is LOWER than best_g at its
#                     cell, and then store its g in best_g. This stops the
#                     search from going around cycles, and keeps A* optimal
#                     when the cost of a step is not always the same (Part 3).
def get_neighbors(n, X_d, best_g, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 3: is_goal(n, X_d) -> True if node n is at cell X_d.
def is_goal(n, X_d):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 4: A_star_planner(X_0, X_d, cost_mode)
#   X_0 = start cell [i0, j0], X_d = goal cell [id, jd].
#   Returns the path P = [X_0, X_1, ..., X_d], a list of cells [i, j], or
#   None if there is no path (the fringe becomes empty).
# Follow GENERAL-SEARCH (Lecture 09A) with the A* strategy (Lecture 09B):
# create the root node, keep a list of nodes on the fringe, pick the node
# with the lowest f(n) from the fringe, check if it is the goal, otherwise
# add its children to the fringe. Build P by following the parents from
# the goal node back to the root. Add any helper functions you want.
def A_star_planner(X_0, X_d, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# PROVIDED - example unit test (empty room, straight line).
def a_star_unit_test_1():
    load_grid(empty_grid())
    X_0, X_d = [25, 25], [125, 25]
    P = A_star_planner(X_0, X_d)
    ok, msg = check_path(P, X_0, X_d)
    optimal_cells = 101             # 100 steps of 1 cell + the start cell
    return ok and len(P) == optimal_cells, P


# STUDENT TODO - Step 5: 2 more unit tests, each returns (passed, P).
# Build a map with empty_grid(), set some cells to 1, call load_grid(...),
# then plan and compare with the optimal path that you computed by hand
# (use check_path and len(P)). Suggested cases:
#   2: a wall of obstacles between X_0 and X_d, with a gap the path must go through
#   3: X_d fully enclosed by obstacles -> A_star_planner must return None
def a_star_unit_test_2():
    pass  # TODO: replace this with your implementation


def a_star_unit_test_3():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 3 tests, print how many pass out of 3, plot the
# map and the 3 paths (e.g. plt.imshow(grid.T, origin="lower")).
def run_a_star_unit_tests(plot=True):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PART 3 - MINIMUM TIME PLANNING WITH A TRAVERSABILITY MAP
# =====================================================================
# In map_4.py, "slow_zones" give each cell a traversability k in (0, 1]:
# the robot may drive at k * ROBOT_SPEED_M_S in that cell.
#
# STUDENT TODO - Step 1: add cost_mode "time" to get_step_cost and get_heuristic.
#   step cost = time (s) to travel one cell into X_to at the speed allowed in X_to
#   heuristic = time (s) to drive the straight line to the goal at the
#               highest possible speed. Explain in your report why it is admissible.
#
# STUDENT TODO - Step 2: 2 unit tests, each returns (passed, P_distance, P_time).
# Use path_time_s(P) to compare the time of two paths.
#   1: a slow zone (k = 0.2) across the straight line from X_0 to X_d, that the
#      robot can go around. "time" must go around it, "distance" goes through.
#   2: the same with k = 0.9: going around is now slower, "time" goes through.
def time_cost_unit_test_1():
    pass  # TODO: replace this with your implementation


def time_cost_unit_test_2():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 2 tests, print how many pass out of 2, plot the paths.
def run_time_cost_unit_tests(plot=True):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PART 4 - FROM PATH TO TRAJECTORY, AND TRAJECTORY TRACKING (Lecture 06A)
# =====================================================================
# A path P only says WHERE the robot goes. A trajectory T also says WHEN:
#   T = [p_0, p_1, ..., p_M],   p_j = [t, x, y, theta]_j   (s, m, m, rad)
# With a trajectory, the robot drives slowly in slow zones, because the
# time stamps there are further apart.
TRAJ_LOOKAHEAD_S = 1.0          # s, the Delta t of Lecture 06A (you may tune it)
HEADING_LOOKAHEAD_CELLS = 10    # cells, see path_to_trajectory


# STUDENT TODO - Step 1: path_to_trajectory(P, cost_mode) -> T
#   P = path of cells from A_star_planner.
#   For each cell, p_j = [t_j, x_j, y_j, theta_j]:
#   - x_j, y_j:  centre of the cell in meters (cell_to_meters)
#   - t_j:       t_0 = 0, then t_j = t_(j-1) + (distance from p_(j-1) to p_j) / speed,
#                with speed = ROBOT_SPEED_M_S for "distance", and
#                k * ROBOT_SPEED_M_S of cell j for "time".
#   - theta_j:   direction of travel. On a grid path the direction of a
#                single step is always 0, 90, 180 or -90 deg, so use the
#                direction from cell j to cell j + HEADING_LOOKAHEAD_CELLS
#                (or to the last cell near the end). The last pose keeps the
#                heading of the pose before it.
def path_to_trajectory(P, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 2: trajectory_tracking_controller(T, X, t) -> (phi_dot_l, phi_dot_r)
#   T = trajectory, X = current pose [x, y, theta], t = time (s) since the
#   robot started the trajectory. Use "Trajectory Tracking Control via
#   Points" of Lecture 06A. At each time step t:
#     1. find the pose p_t on T that has the same time stamp as t
#     2. find the pose p_des,t on T that is a time TRAJ_LOOKAHEAD_S further
#        along T (the last pose of T if t + TRAJ_LOOKAHEAD_S is after the end)
#     3. return point_tracking_controller([x, y, theta] of p_des,t, X)
def trajectory_tracking_controller(T, X, t):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 3: 2 unit tests, each returns (passed, T, visited_poses).
# Plan on a map of map_4.py with load_map(name) and A_star_planner, make the
# trajectory, then loop: trajectory_tracking_controller -> robot_simulator
# (t increases by DT each loop) until is_at_goal(T[-1][1:], X), with a time
# limit of T[-1][0] + 20 s. A test passes if the robot reaches the goal and
# robot_in_collision is False for every pose visited. Also record the largest
# distance between the robot and p_t (the tracking error) and print it.
#   1: map "box_sim", X = [0, 0, 0], goal (1.5, 0.0) m, cost_mode "distance"
#   2: map "terrain_sim", X = [0, 0, 0], goal (1.5, 0.0) m, cost_mode "time"
def traj_tracker_unit_test_1():
    pass  # TODO: replace this with your implementation


def traj_tracker_unit_test_2():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 2 tests, print how many pass out of 2, plot each
# planned path and the poses visited (as subfigures of one figure).
def run_traj_tracker_unit_tests(plot=True):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PART 5 - A* ANALYSIS: PATH LENGTH AND PLANNING TIME VS MAP DENSITY
# =====================================================================
# PROVIDED - a random free cell, for random start and goal cells.
def random_free_cell(rng):
    while True:
        X = [int(rng.integers(0, GRID_N)), int(rng.integers(0, GRID_N))]
        if not is_occupied(X):
            return X


# STUDENT TODO: run_density_analysis(plot=True)
#   For 5 densities (e.g. 0.0, 0.05, 0.10, 0.15, 0.20):
#     load_grid(map_4.random_map(density, seed))
#     100 times: X_0 = random_free_cell(rng), X_d = random_free_cell(rng),
#       time the call A_star_planner(X_0, X_d) with time.perf_counter(),
#       record the path length (m) and the planning time (s).
#       Count the plans that return None (no path) separately.
#   Plot the mean path length vs density, and the mean planning time vs density.
# Tip: with a plain list as the fringe, searching it for the lowest f at every
# step makes this analysis slow (~10 min). Python's heapq module can give you
# the lowest f much faster. Mention your choice in the report.
def run_density_analysis(plot=True):
    pass  # TODO: replace this with your implementation


if __name__ == "__main__":
    print("=== Part 2: A* (shortest path) ===")
    run_a_star_unit_tests()
    print("\n=== Part 3: A* (minimum time) ===")
    run_time_cost_unit_tests()
    print("\n=== Part 4: trajectory tracking ===")
    run_traj_tracker_unit_tests()
    # print("\n=== Part 5: density analysis ===")   # slow: run it once for your report
    # run_density_analysis()
