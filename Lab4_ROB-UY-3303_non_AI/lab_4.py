"""
ROB-UY 3303 - Lab 04: Tree Search Motion Planning (A*)
GUIDED STARTER CODE - complete every function marked STUDENT TODO.

    Run the checks and unit tests with:   python lab_4.py

The GUI (gui_4.py) imports this file and uses your planner and trackers on
the robot. map_4.py (same folder) holds the maps.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them.

HOW TO WORK THROUGH THIS FILE
  Do the parts in order. After EVERY function you write, run
      python lab_4.py
  The SELF-CHECKS at the top of the output test each function on its own
  and tell you what was expected and what your function returned. Fix the
  first failing check before moving on.

Units: meters, seconds, radians.
Grid: 200 x 200 cells of 1 cm (see map_4.py). A cell is a list [i, j] of
two ints: i = column (along x), j = row (along y).

            j (y)
            ^
        199 |  .  .  .  .
            |  .  .  .  .        cell [i, j] has 4 neighbors:
          2 |  .  .  .  .            [i+1, j]   (+x)
          1 |  .  .  .  .            [i-1, j]   (-x)
          0 |  .  .  .  .            [i, j+1]   (+y)
            +-------------> i (x)    [i, j-1]   (-y)
               0  1  2 ... 199

-----------------------------------------------------------------------
QUICK PYTHON REMINDERS (you have no internet: everything you need is here)
-----------------------------------------------------------------------
  Lists:      a = []            empty list
              a.append(x)       add x at the end
              len(a)            number of elements
              a[-1]             last element
              a.pop(k)          remove AND return element number k
              a[::-1]           a copy of the list, reversed
              [x, y] == [x, y]  True: lists can be compared with ==
  Loops:      for k in range(len(a)):        k = 0, 1, ..., len(a) - 1
              for p in a:                    p = each element of a
              while condition:               repeat while condition is True
  Math:       math.hypot(dx, dy)  = sqrt(dx^2 + dy^2)
              math.atan2(dy, dx)  = angle of the vector (dx, dy), in (-pi, pi]
              math.inf            = infinity (bigger than any number)
  Classes:    class Node:
                  def __init__(self, a, b):
                      self.a = a              # store a member variable
              n = Node(1, 2)                  # create a Node
              n.a                             # read a member variable
  None:       x is None           True if x is None (e.g. "no parent")
  2D arrays:  grid[i][j]          value of cell [i, j]
"""
import heapq
import math
import time

import numpy as np

import map_4

# =====================================================================
# PART 0 - STUDENT TODO: COPY YOUR LAB 3 CODE (lab_3.py)
# =====================================================================
# Open your lab_3.py and copy:
#   - your two robot constants (the same values as in gui_4.py)
#   - get_wheel_velocities(v, w)
#   - point_tracking_controller(X_des, X)
#   - path_tracking_controller(P, X)
# Keep the same names, inputs and outputs. If these functions call helper
# functions that you wrote in lab_3.py, copy those helpers too.
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
# You will use these functions. Read their descriptions.
GRID_N = map_4.GRID_N                   # 200 cells along x and along y
CELL_SIZE_M = map_4.CELL_SIZE_M         # 0.01 m
ROBOT_SPEED_M_S = 0.12                  # m/s, robot speed on normal floor (k = 1)

OCCUPANCY_GRID = None       # 200 x 200, 1 = robot centre cannot be in the cell
TRAVERSABILITY_GRID = None  # 200 x 200, k in (0, 1]: speed allowed = k * ROBOT_SPEED_M_S
COLLISION_GRID = None       # 200 x 200, 1 = the robot body touches an obstacle or a wall


def load_map(name):
    """Loads a map of map_4.MAPS: the obstacles grown by the robot radius
    and a safety margin (configuration space, Lecture 08A: "grow" the
    obstacles so that we can plan for a point robot), and the traversability."""
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
    """200 x 200 grid of zeros (no obstacles). To add an obstacle in cell
    [i, j]:  grid[i][j] = 1.  To fill a rectangle of cells:
    grid[i_min:i_max, j_min:j_max] = 1  (i_max and j_max NOT included)."""
    return np.zeros((GRID_N, GRID_N), dtype=np.uint8)


def in_bounds(X):
    """True if cell X = [i, j] is on the grid (0 <= i < 200 and 0 <= j < 200)."""
    return 0 <= X[0] < GRID_N and 0 <= X[1] < GRID_N


def is_occupied(X):
    """True if cell X = [i, j] is in collision. Call in_bounds(X) FIRST:
    a cell off the grid does not exist in the array."""
    return OCCUPANCY_GRID[X[0]][X[1]] == 1


def traversability(X):
    """Traversability k of cell X = [i, j]: 1.0 = normal floor, 0.2 = the
    robot may only drive at 20 % of ROBOT_SPEED_M_S in this cell."""
    return float(TRAVERSABILITY_GRID[X[0]][X[1]])


def new_cost_array():
    """200 x 200 array full of math.inf (one value per cell).
    Read cell [i, j]:  a[i][j]      Write cell [i, j]:  a[i][j] = value"""
    return np.full((GRID_N, GRID_N), math.inf)


def meters_to_cell(x, y):
    """Position (m) -> cell [i, j] that contains it.
    Example: the robot start (0.0, 0.0) m is cell [25, 25]."""
    return [int(math.floor((x - map_4.GRID_ORIGIN_M[0]) / CELL_SIZE_M)),
            int(math.floor((y - map_4.GRID_ORIGIN_M[1]) / CELL_SIZE_M))]


def cell_to_meters(X):
    """Cell [i, j] -> position (m) [x, y] of its centre.
    Example: cell [25, 25] -> [0.005, 0.005] m."""
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
# Two cost functions are used in this lab (the input cost_mode):
#   "distance": shortest path. The cost of a step is the distance (m). Part 2.
#   "time":     minimum time. The cost of a step is the time (s). Part 3.
# For now, write everything for "distance" only. You add "time" in Part 3.
#
# Reminder from Lecture 09B, A* search:
#     f(n) = g(n) + h(n)
#     g(n) = cost of the path from the start (root) to node n
#     h(n) = ESTIMATED cost from node n to the goal
#     h must be ADMISSIBLE: it never overestimates the real cost to the goal.
#     Example of admissible h: the straight line distance from n to the goal.


# =====================================================================
# STUDENT TODO - Step 1a: the Node class
# =====================================================================
# Purpose: one node of the search tree.
#
# Parameters of __init__:
#   X        (list)  - [i, j], the cell of the node
#   parent   (Node)  - the parent node (None for the root of the tree)
#   g_value  (float) - g(n), path cost from the root to this node
#   h_value  (float) - h(n), estimated path cost from this node to the goal
#
# Store all 4 as member variables with the SAME names (self.X,
# self.parent, self.g_value, self.h_value), and add a 5th one:
#   self.f_value = f(n) = g(n) + h(n)
#
# Self-check: Node([3, 4], None, 1.0, 2.0).f_value must be 3.0
# =====================================================================
class Node:
    def __init__(self, X, parent, g_value, h_value):
        pass  # TODO: replace this with your implementation (5 lines)


# =====================================================================
# STUDENT TODO - Step 1b: get_step_cost
# =====================================================================
# Purpose: cost of moving from cell X_from to the neighbor cell X_to.
#
# Parameters:
#   X_from, X_to (list) - two neighbor cells [i, j]
#   cost_mode    (str)  - "distance" (Part 2) or "time" (Part 3)
#
# Returns:
#   "distance": the distance travelled, in METERS.
#
#       cost = CELL_SIZE_M * sqrt( (i_to - i_from)^2 + (j_to - j_from)^2 )
#
#       Neighbors are 1 cell apart, so this is always 0.01 m, but write the
#       general equation.
#
#   "time" (Part 3, leave it for later): the time in SECONDS to drive
#       that distance at the speed allowed in X_to:
#
#       speed = traversability(X_to) * ROBOT_SPEED_M_S        (m/s)
#       cost  = distance / speed                              (s)
#
# Self-checks:
#   get_step_cost([0, 0], [1, 0], "distance") = 0.01
#   with k = 0.5 in cell [1, 0]:
#   get_step_cost([0, 0], [1, 0], "time") = 0.01 / (0.5 * 0.12) = 0.1667
#
# Hint: math.hypot(di, dj) = sqrt(di^2 + dj^2)
# =====================================================================
def get_step_cost(X_from, X_to, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 1c: get_heuristic
# =====================================================================
# Purpose: h(n), the estimated cost from cell X to the goal cell X_d.
#
# Returns:
#   "distance": the STRAIGHT LINE distance from X to X_d, in METERS
#               (the admissible heuristic of Lecture 09B):
#
#       h = CELL_SIZE_M * sqrt( (i_d - i)^2 + (j_d - j)^2 )
#
#       Why admissible? The robot moves in steps of +-x and +-y, so any
#       path to the goal is AT LEAST as long as the straight line.
#
#   "time" (Part 3, leave it for later): the time to drive the straight
#       line at the HIGHEST speed the robot can have (k = 1):
#
#       h = straight line distance / ROBOT_SPEED_M_S             (s)
#
#       Why admissible? No cell lets the robot drive faster than
#       ROBOT_SPEED_M_S, and no path is shorter than the straight line.
#
# Self-checks:
#   get_heuristic([0, 0], [3, 4], "distance") = 0.01 * 5 = 0.05
#   get_heuristic([0, 0], [3, 4], "time")     = 0.05 / 0.12 = 0.4167
# =====================================================================
def get_heuristic(X, X_d, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 2: get_neighbors
# =====================================================================
# Purpose: create the children of node n in the search tree: the cells
#          one step in +x, -x, +y and -y from n.X.
#
# Parameters:
#   n         (Node)  - the node being expanded
#   X_d       (list)  - goal cell [i, j] (needed to compute h of each child)
#   best_g    (array) - 200 x 200 array, best_g[i][j] = lowest g found so
#                       far for cell [i, j] (math.inf if never reached).
#                       It is created ONCE in A_star_planner and passed here.
#   cost_mode (str)   - "distance" or "time"
#
# Returns:
#   children (list of Node) - 0 to 4 new nodes
#
# What to do, for each of the 4 neighbor cells of n.X:
#   - Do NOT create the neighbor if it is:
#       out of bounds:  off the grid             (in_bounds)
#       in collision:   an occupied cell         (is_occupied)
#       already added:  see best_g below
#   - Otherwise create a Node for it, with the right parent, g and h
#     (f is computed by the Node class).
#
# Equation for the g of a child (Lecture 09B: g = cost from the start):
#     g(child) = g(n) + cost of the step from n to the child
#
# The "already added" check with best_g:
#   Only create the child if its g is LOWER than the value stored in best_g
#   for its cell, and then store its g there. Without this check the search
#   goes around in cycles forever (A -> B -> A -> B ...). A cell reached
#   again by a CHEAPER path is still allowed: this keeps A* optimal in
#   Part 3, where a step costs more in the slow zones.
#
# Hint: a loop over the 4 moves, e.g. [(1, 0), (-1, 0), (0, 1), (0, -1)],
#       avoids writing the same code 4 times. "continue" skips to the next move.
#
# Self-check: on an empty grid, the node at [0, 0] has only 2 children,
#             [1, 0] and [0, 1] (the cells [-1, 0] and [0, -1] are off the grid).
# =====================================================================
def get_neighbors(n, X_d, best_g, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 3: is_goal
# =====================================================================
# Returns True if node n is located at the goal cell X_d.
# Hint: compare both indices, or compare the lists with ==.
# =====================================================================
def is_goal(n, X_d):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PROVIDED - the fringe, for Step 4
# =====================================================================
class Fringe:
    """The fringe of the search tree (Lecture 09A): the nodes created but
    not expanded yet. It always gives back the node with the LOWEST f_value
    first, which is the A* strategy (Lecture 09B).

        fringe = Fringe()               create an empty fringe
        fringe.add(n)                   add one node
        fringe.add_all(children)        add a list of nodes
        fringe.is_empty()               True if there is no node left
        n = fringe.pop_lowest_f()       remove and return the node with the lowest f
    """
    def __init__(self):
        self._heap = []                 # a "heap": always finds the lowest f quickly
        self._count = 0

    def add(self, n):
        heapq.heappush(self._heap, (n.f_value, self._count, n))
        self._count += 1

    def add_all(self, nodes):
        for n in nodes:
            self.add(n)

    def is_empty(self):
        return len(self._heap) == 0

    def pop_lowest_f(self):
        return heapq.heappop(self._heap)[2]

    def __len__(self):
        return len(self._heap)


# =====================================================================
# STUDENT TODO - Step 4a: build_path
# =====================================================================
# Purpose: once the goal node is found, build the path from the start.
#
# Parameters:
#   n (Node) - the goal node
#
# Returns:
#   P (list of cells) - [X_0, X_1, ..., X_d], from the ROOT to n
#
# Idea: every node knows its parent, and the root is the only node whose
# parent is None. Starting at the goal node, collect the cells while
# going up the tree until you have passed the root. You then have the
# path from the goal to the start, so it still needs to be reversed.
#
# Example: root [0,0] -> [1,0] -> [1,1] = n.  P must be [[0,0], [1,0], [1,1]]
# =====================================================================
def build_path(n):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 4b: A_star_planner
# =====================================================================
# Purpose: plan a path from cell X_0 to cell X_d.
#
# Parameters:
#   X_0       (list) - start cell [i0, j0]
#   X_d       (list) - goal cell [id, jd]
#   cost_mode (str)  - "distance" or "time"
#
# Returns:
#   P = [X_0, X_1, ..., X_d] (list of cells), or None if there is no path.
#
# This is GENERAL-SEARCH of Lecture 09A, with the A* strategy of 09B:
#
#   function GENERAL-SEARCH(problem, strategy)
#       add root node to fringe
#       loop do
#           if fringe is empty
#               return failure
#           pick node n from fringe via strategy
#           if the node n contains a goal
#               return path from root to n
#           else
#               add node n's children to fringe
#       end
#
# Each line of the pseudo code uses one of the tools you already have:
#   root node                    -> a Node at X_0 with no parent. What is its g?
#   fringe                       -> the provided Fringe class (above)
#   pick node n via strategy     -> A* picks the node with the lowest f(n)
#   node n contains a goal       -> is_goal
#   path from root to n          -> build_path
#   node n's children            -> get_neighbors
#   failure                      -> return None
#
# Before the loop, also create best_g (new_cost_array) for get_neighbors,
# and store in it the g of the start cell.
#
# Note: check is_goal when a node is PICKED from the fringe, not when it
# is created. This is what makes the path optimal.
# =====================================================================
def A_star_planner(X_0, X_d, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PART 2 - UNIT TESTS
# =====================================================================
# PROVIDED - unit test 1: empty room, straight line.
# The optimal path from [25, 25] to [125, 25] is 100 steps along +x,
# so it has 101 cells (100 steps + the start cell).
def a_star_unit_test_1():
    load_grid(empty_grid())
    X_0, X_d = [25, 25], [125, 25]
    P = A_star_planner(X_0, X_d)
    ok, msg = check_path(P, X_0, X_d)
    return ok and len(P) == 101, P


# PROVIDED - unit test 2: a wall with a gap.
# Wall of obstacles at i = 100 for j = 0 ... 149 (gap above j = 150).
# From [50, 50] to [150, 50] the robot must go up to j = 150, cross, and
# come back down. Optimal number of steps (computed by hand):
#   100 steps along x + (150 - 50) steps up + (150 - 50) steps down = 300
# so the path has 301 cells.
def a_star_unit_test_2():
    grid = empty_grid()
    grid[100, 0:150] = 1
    load_grid(grid)
    X_0, X_d = [50, 50], [150, 50]
    P = A_star_planner(X_0, X_d)
    ok, msg = check_path(P, X_0, X_d)
    return ok and len(P) == 301, P


# =====================================================================
# STUDENT TODO - unit test 3: the goal cannot be reached.
# =====================================================================
# Build a map where the goal (e.g. [150, 150]) is inside a CLOSED box of
# obstacles (4 sides, no gap), plan from a cell outside the box, and check
# that A_star_planner returns None. Look at unit test 2 for how to build a
# map, and at empty_grid() for how to fill a row of cells at once.
# Return (passed, P), like the other tests.
# =====================================================================
def a_star_unit_test_3():
    pass  # TODO: replace this with your implementation


# PROVIDED - runs the 3 tests, prints the result and plots the paths.
def run_a_star_unit_tests(plot=True):
    tests = [a_star_unit_test_1, a_star_unit_test_2, a_star_unit_test_3]
    results = []
    for k, test in enumerate(tests):
        out = test()
        if out is None:
            print(f"  test {k + 1}: not written yet")
            continue
        passed, P = out
        print(f"  test {k + 1}: {'pass' if passed else 'FAIL'}")
        results.append((k + 1, passed, P, OCCUPANCY_GRID.copy()))
    print(sum(r[1] for r in results), "out of", len(tests), "tests passed")
    if plot and results:
        plot_grid_paths([(f"A* test {k}: {'pass' if ok else 'FAIL'}", grid, None, [P] if P else [])
                         for k, ok, P, grid in results])


# =====================================================================
# PART 3 - MINIMUM TIME PLANNING WITH A TRAVERSABILITY MAP
# =====================================================================
# Lecture 08A: a path can be optimal for different metrics: "shortest
# path, minimum time, lowest energy". In map_4.py, "slow_zones" give each
# cell a traversability k in (0, 1]: the robot may drive at
#     speed = k * ROBOT_SPEED_M_S
# in that cell (think of sand, carpet, or a crowded area).
#
# STUDENT TODO - Step 1: go back to get_step_cost and get_heuristic and
# add the "time" equations written in their comments. Nothing else in
# your planner changes: A* now minimizes the TOTAL TIME instead of the
# total distance.
#
#   total time of a path = sum over its steps of  distance / (k * ROBOT_SPEED_M_S)


# PROVIDED - a test map: a slow strip with traversability k across the
# straight line from [25, 50] to [175, 50]. The robot can go around the
# strip above j = 120 (a detour of 2 x 70 = 140 extra cells).
def _slow_strip_test(k):
    trav = np.ones((GRID_N, GRID_N))
    trav[80:120, 0:120] = k                         # 40 cells wide, j = 0 ... 119
    load_grid(empty_grid(), trav)
    X_0, X_d = [25, 50], [175, 50]
    P_distance = A_star_planner(X_0, X_d, "distance")
    P_time = A_star_planner(X_0, X_d, "time")
    return P_distance, P_time


# PROVIDED - time test 1: k = 0.2. Times computed by hand:
#   straight through: 110 normal cells + 40 slow cells
#     = 110 * 0.01 / 0.12 + 40 * 0.01 / (0.2 * 0.12) = 9.2 + 16.7 = 25.8 s
#   around the strip: 150 + 140 = 290 normal cells = 290 * 0.01 / 0.12 = 24.2 s
#   -> "time" must go AROUND (and be faster), "distance" goes THROUGH.
def time_cost_unit_test_1():
    P_d, P_t = _slow_strip_test(0.2)
    if P_d is None or P_t is None:
        return False, P_d, P_t
    print(f"  test 1: distance path {path_time_s(P_d):.1f} s, time path {path_time_s(P_t):.1f} s")
    went_around = max(X[1] for X in P_t) >= 120
    passed = went_around and len(P_d) == 151 and path_time_s(P_t) < path_time_s(P_d)
    return passed, P_d, P_t


# =====================================================================
# STUDENT TODO - time test 2: k = 0.9
# =====================================================================
# Compute by hand, like test 1, the time to go straight through the strip
# and the time to go around it, with k = 0.9. Write them in a comment.
# Which path should "time" choose now? Write the test like test 1, with
# the pass condition that matches your answer. Return (passed, P_d, P_t).
# =====================================================================
def time_cost_unit_test_2():
    pass  # TODO: replace this with your implementation


# PROVIDED - runs the 2 tests, prints the result and plots the paths.
def run_time_cost_unit_tests(plot=True):
    results = []
    for k, (test, strip_k) in enumerate(((time_cost_unit_test_1, 0.2), (time_cost_unit_test_2, 0.9))):
        out = test()
        if out is None:
            print(f"  test {k + 1}: not written yet")
            continue
        passed, P_d, P_t = out
        print(f"  test {k + 1}: {'pass' if passed else 'FAIL'}")
        trav = np.ones((GRID_N, GRID_N))
        trav[80:120, 0:120] = strip_k
        results.append((f"time test {k + 1} (k = {strip_k}): {'pass' if passed else 'FAIL'}",
                        empty_grid(), trav, [P for P in (P_d, P_t) if P]))
        results[-1] = results[-1] + (passed,)
    print(sum(r[4] for r in results), "out of 2 tests passed")
    if plot and results:
        plot_grid_paths([r[:4] for r in results], labels=["distance", "time"])


# =====================================================================
# PART 4 - FROM PATH TO TRAJECTORY, AND TRAJECTORY TRACKING (Lecture 06A)
# =====================================================================
# A path P only says WHERE the robot goes. A trajectory T also says WHEN:
#   T = [p_0, p_1, ..., p_M],   p_j = [t, x, y, theta]_j   (s, m, m, rad)
# With a trajectory the robot drives slowly in the slow zones, because the
# time stamps there are further apart.
TRAJ_LOOKAHEAD_S = 1.0          # s, the Delta t of Lecture 06A (you may tune it)
HEADING_LOOKAHEAD_CELLS = 10    # cells, see path_to_trajectory


# =====================================================================
# STUDENT TODO - Step 1: path_to_trajectory
# =====================================================================
# Purpose: add a position in meters, a time stamp and a heading to each
#          cell of the path.
#
# Parameters:
#   P         (list) - path of cells from A_star_planner, [[i, j], ...]
#   cost_mode (str)  - "distance" or "time"
#
# Returns:
#   T (list) - [[t_0, x_0, y_0, theta_0], [t_1, x_1, y_1, theta_1], ...]
#              one pose for each cell of P (same length as P)
#
# Equations, for j = 0, 1, ..., len(P) - 1:
#   position:  [x_j, y_j] = cell_to_meters(P[j])
#   time:      t_0 = 0
#              t_j = t_(j-1) + d_j / s_j          for j >= 1
#                 d_j = distance from [x_(j-1), y_(j-1)] to [x_j, y_j]  (0.01 m)
#                 s_j = ROBOT_SPEED_M_S                               ("distance")
#                 s_j = traversability(P[j]) * ROBOT_SPEED_M_S        ("time")
#   heading:   on a grid path each single step points at 0, 90, 180 or
#              -90 deg, even on a diagonal "staircase". So look further:
#                 m = min(j + HEADING_LOOKAHEAD_CELLS, len(P) - 1)
#                 theta_j = atan2(y_m - y_j, x_m - x_j)
#              For the LAST cell (m == j) use the heading of the pose before it.
#
# Worked example ("distance", ROBOT_SPEED_M_S = 0.12 m/s):
#   P = [[25, 25], [26, 25], [27, 25]]
#   T = [[0.0,    0.005, 0.005, 0.0],
#        [0.0833, 0.015, 0.005, 0.0],        0.01 m / 0.12 m/s = 0.0833 s
#        [0.1667, 0.025, 0.005, 0.0]]
# =====================================================================
def path_to_trajectory(P, cost_mode="distance"):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 2: trajectory_tracking_controller
# =====================================================================
# Purpose: "Trajectory Tracking Control via Points" (Lecture 06A).
#
# Parameters:
#   T (list)  - trajectory from path_to_trajectory
#   X (list)  - current robot pose [x, y, theta]
#   t (float) - time (s) since the robot started the trajectory
#
# Returns:
#   (phi_dot_l, phi_dot_r) - wheel speeds from your point tracking controller
#
# Lecture 06A, at each time step t:
#   1. Find the pose p_t on T that has the same time stamp as t.
#   2. Find the pose p_des,t on T that is a time Delta t = TRAJ_LOOKAHEAD_S
#      further along T than p_t.
#   3. Calculate the control signals using p_des,t as the input to the
#      point tracking controller.
#
# Hints:
#   - The time stamp of pose T[j] is T[j][0]. The time stamps grow along T.
#   - Steps 1 and 2 together: p_des,t is the first pose of T whose time
#     stamp is at least t + TRAJ_LOOKAHEAD_S.
#   - Near the end there may be no such pose: then use the last pose of T,
#     so the robot drives to the end of the trajectory and stops there.
#   - A pose of T is [t, x, y, theta], but your point tracker wants
#     [x, y, theta].
#   - Compare with your path_tracking_controller of Lab 3: the same idea,
#     with a time Delta t instead of a distance Delta.
# =====================================================================
def trajectory_tracking_controller(T, X, t):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PROVIDED - simulates the robot following a trajectory (Lab 3 style loop)
# =====================================================================
def simulate_trajectory(T, X):
    """Runs trajectory_tracking_controller -> robot_simulator every DT
    seconds until the robot is at the end of T, or T[-1][0] + 20 s.
    Returns (reached, visited_poses, time_taken_s, max_tracking_error_m)."""
    t, max_err = 0.0, 0.0
    visited = [list(X)]
    while not is_at_goal(T[-1][1:], X) and t < T[-1][0] + 20.0:
        phi_l, phi_r = trajectory_tracking_controller(T, X, t)
        X, poses = robot_simulator(X, phi_l, phi_r, DT)
        visited += poses
        t += DT
        p_t = next((p for p in T if p[0] >= t), T[-1])          # pose with time stamp t
        max_err = max(max_err, math.hypot(p_t[1] - X[0], p_t[2] - X[1]))
    return is_at_goal(T[-1][1:], X), visited, t, max_err


# PROVIDED - trajectory test 1: map "box_sim", from (0, 0) to (1.5, 0.0) m,
# shortest path. The robot must reach the goal without touching the box.
def traj_tracker_unit_test_1():
    load_map("box_sim")
    X = [0.0, 0.0, 0.0]
    P = A_star_planner(meters_to_cell(X[0], X[1]), meters_to_cell(1.5, 0.0), "distance")
    T = path_to_trajectory(P, "distance") if P else None
    if not T:
        print("  box_sim: A_star_planner or path_to_trajectory not written yet")
        return False, [], []
    reached, visited, t, max_err = simulate_trajectory(T, X)
    hit = any(robot_in_collision(p) for p in visited)
    print(f"  box_sim: planned {T[-1][0]:.1f} s, took {t:.1f} s, "
          f"max tracking error {max_err:.3f} m, collision {hit}")
    return reached and not hit, T, visited


# =====================================================================
# STUDENT TODO - trajectory test 2: map "terrain_sim", minimum time.
# =====================================================================
# Same as traj_tracker_unit_test_1, from (0, 0) to (1.5, 0.0) m, but on the
# map "terrain_sim" with the minimum time cost. Think about every place
# where the cost mode must change. Return (passed, T, visited_poses).
# In your report, compare the planned time and the time taken with the
# same goal planned with "distance" on this map.
# =====================================================================
def traj_tracker_unit_test_2():
    pass  # TODO: replace this with your implementation


# PROVIDED - runs the 2 tests, prints the result and plots the paths.
def run_traj_tracker_unit_tests(plot=True):
    results = []
    for k, (test, name) in enumerate(((traj_tracker_unit_test_1, "box_sim"),
                                      (traj_tracker_unit_test_2, "terrain_sim"))):
        out = test()
        if out is None:
            print(f"  test {k + 1}: not written yet")
            continue
        passed, T, visited = out
        print(f"  test {k + 1}: {'pass' if passed else 'FAIL'}")
        results.append((name, passed, T, visited))
    print(sum(r[1] for r in results), "out of 2 tests passed")
    if plot and results:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, len(results), figsize=(5 * len(results), 5), squeeze=False)
        ext = [-0.25, 1.75, -0.25, 1.75]
        for ax, (name, passed, T, visited) in zip(axes[0], results):
            ax.imshow(map_4.build_traversability_grid(name).T, origin="lower", cmap="YlOrBr_r",
                      extent=ext, vmin=0, vmax=1)
            occ = np.ma.masked_equal(map_4.build_obstacle_grid(name).T, 0)
            ax.imshow(occ, origin="lower", cmap="Greys_r", extent=ext)
            ax.plot([p[1] for p in T], [p[2] for p in T], "b--", label="planned")
            ax.plot([p[0] for p in visited], [p[1] for p in visited], "r-", label="robot")
            ax.set_title(f"{name}: {'pass' if passed else 'FAIL'}")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.legend()
        plt.tight_layout()
        plt.show()


# =====================================================================
# PART 5 - A* ANALYSIS: PATH LENGTH AND PLANNING TIME VS MAP DENSITY
# =====================================================================
# PROVIDED - a random free cell, for random start and goal cells.
def random_free_cell(rng):
    while True:
        X = [int(rng.integers(0, GRID_N)), int(rng.integers(0, GRID_N))]
        if not is_occupied(X):
            return X


# =====================================================================
# STUDENT TODO: run_density_analysis
# =====================================================================
# Purpose: measure how the map density (fraction of cells occupied)
#          changes the path length and the planning time of A*.
#
# What to do:
#   For 5 densities of increasing value (e.g. 0.0, 0.05, 0.10, 0.15, 0.20):
#     - load a random map of that density
#     - 100 times: pick a random start cell and a random goal cell, plan,
#       and record the path length (m) and the planning time (s)
#     - compute the mean path length and the mean planning time
#   Then plot both means against the density.
#
# Tools you can use:
#   map_4.random_map(density, seed)  -> a random 200 x 200 obstacle grid
#   load_grid(grid)                  -> load it for the planner
#   rng = np.random.default_rng(0)   -> a random number generator (create it once)
#   random_free_cell(rng)            -> a random cell that is not occupied
#   time.perf_counter()              -> a clock in seconds: call it before and
#                                       after A_star_planner, and subtract
#   plot_density_results(densities, mean_lengths, mean_times) -> the 2 plots
#
# Equation: a path of N cells has N - 1 steps of CELL_SIZE_M, so its length is
#     length = (N - 1) * CELL_SIZE_M        (m)
#
# Careful: A_star_planner can return None (start or goal cut off by the
# obstacles). Do not add these plans to the mean length; count them, and
# report the count.
#
# It takes about 1 minute: each plan takes up to ~1 s.
# =====================================================================
def run_density_analysis(plot=True):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PROVIDED - PLOTTING (do not change)
# =====================================================================
def plot_grid_paths(panels, labels=None):
    """panels: list of (title, occupancy grid, traversability grid or None,
    list of paths of cells). One subfigure per panel."""
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(panels), figsize=(4.5 * len(panels), 4.5), squeeze=False)
    styles = ["b-", "r--", "g-.", "m:"]
    for ax, (title, grid, trav, paths) in zip(axes[0], panels):
        if trav is not None:
            ax.imshow(trav.T, origin="lower", cmap="YlOrBr_r", vmin=0, vmax=1)
        ax.imshow(np.ma.masked_equal(np.asarray(grid).T, 0), origin="lower", cmap="Greys_r")
        for k, P in enumerate(paths):
            label = labels[k] if labels and k < len(labels) else None
            ax.plot([X[0] for X in P], [X[1] for X in P], styles[k % 4], label=label)
        ax.set_xlim(-0.5, GRID_N - 0.5)
        ax.set_ylim(-0.5, GRID_N - 0.5)
        ax.set_title(title)
        ax.set_xlabel("i (cell)")
        ax.set_ylabel("j (cell)")
        if labels and paths:
            ax.legend()
    plt.tight_layout()
    plt.show()


def plot_density_results(densities, mean_lengths, mean_times):
    """Plots the mean path length and the mean planning time vs density."""
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
    a1.plot(densities, mean_lengths, "o-")
    a1.set_xlabel("map density (fraction of occupied cells)")
    a1.set_ylabel("mean path length (m)")
    a2.plot(densities, mean_times, "o-")
    a2.set_xlabel("map density (fraction of occupied cells)")
    a2.set_ylabel("mean planning time (s)")
    plt.tight_layout()
    plt.show()


# =====================================================================
# PROVIDED - SELF-CHECKS (do not change)
# =====================================================================
# Each check tests ONE function with an answer computed by hand, and tells
# you what is wrong. Run "python lab_4.py" after each function you write.
def _check(name, fn):
    try:
        msg = fn()
    except Exception as e:                          # show the error, keep going
        msg = f"error: {type(e).__name__}: {e}"
    print(f"  [{'ok' if msg is None else '--'}] {name}" + ("" if msg is None else f": {msg}"))
    return msg is None


def _close(a, b, tol=1e-4):
    return a is not None and abs(a - b) < tol


def _check_node():
    n = Node([3, 4], None, 1.0, 2.0)
    for attr in ("X", "parent", "g_value", "h_value", "f_value"):
        if not hasattr(n, attr):
            return f"Node has no member variable self.{attr}"
    if not _close(n.f_value, 3.0):
        return f"Node([3, 4], None, 1.0, 2.0).f_value should be 3.0, got {n.f_value}"


def _check_step_cost():
    trav = np.ones((GRID_N, GRID_N))
    trav[1][0] = 0.5
    load_grid(empty_grid(), trav)
    c = get_step_cost([0, 0], [1, 0], "distance")
    if not _close(c, 0.01):
        return f'get_step_cost([0, 0], [1, 0], "distance") should be 0.01 (m), got {c}'
    c = get_step_cost([0, 0], [1, 0], "time")
    if not _close(c, 0.01 / (0.5 * 0.12)):
        return f'get_step_cost([0, 0], [1, 0], "time") with k = 0.5 should be 0.1667 (s), got {c} (Part 3)'


def _check_heuristic():
    h = get_heuristic([0, 0], [3, 4], "distance")
    if not _close(h, 0.05):
        return f'get_heuristic([0, 0], [3, 4], "distance") should be 0.05 (m), got {h}'
    h = get_heuristic([0, 0], [3, 4], "time")
    if not _close(h, 0.05 / 0.12):
        return f'get_heuristic([0, 0], [3, 4], "time") should be 0.4167 (s), got {h} (Part 3)'


def _check_neighbors():
    load_grid(empty_grid())
    root = Node([0, 0], None, 0.0, 0.0)
    best_g = new_cost_array()
    best_g[0][0] = 0.0
    kids = get_neighbors(root, [10, 10], best_g)
    if kids is None:
        return "get_neighbors returned None: return a list"
    cells = sorted(list(k.X) for k in kids)
    if cells != [[0, 1], [1, 0]]:
        return f"children of [0, 0] on an empty grid should be [0, 1] and [1, 0], got {cells}"
    for k in kids:
        if k.parent is not root:
            return "the parent of each child must be the node n"
        if not _close(k.g_value, 0.01):
            return f"g of a child of the root should be 0 + 0.01 = 0.01, got {k.g_value}"
        if not _close(k.h_value, get_heuristic(k.X, [10, 10])):
            return "h of a child must be get_heuristic(child cell, X_d, cost_mode)"
    kids = get_neighbors(root, [10, 10], best_g)
    if kids:
        return "the same children were created twice: use best_g for the 'already added' check"
    grid = empty_grid()
    grid[1][0] = 1
    load_grid(grid)
    best_g = new_cost_array()
    cells = sorted(list(k.X) for k in get_neighbors(root, [10, 10], best_g))
    if cells != [[0, 1]]:
        return f"cell [1, 0] is occupied: the only child of [0, 0] should be [0, 1], got {cells}"


def _check_is_goal():
    if is_goal(Node([5, 6], None, 0.0, 0.0), [5, 6]) is not True:
        return "is_goal(node at [5, 6], [5, 6]) should be True"
    if is_goal(Node([5, 6], None, 0.0, 0.0), [6, 5]) is not False:
        return "is_goal(node at [5, 6], [6, 5]) should be False"


def _check_build_path():
    a = Node([0, 0], None, 0.0, 0.0)
    b = Node([1, 0], a, 0.01, 0.0)
    c = Node([1, 1], b, 0.02, 0.0)
    P = build_path(c)
    if P is None or [list(X) for X in P] != [[0, 0], [1, 0], [1, 1]]:
        return f"build_path should return [[0, 0], [1, 0], [1, 1]] (root first), got {P}"


def _check_a_star_small():
    load_grid(empty_grid())
    P = A_star_planner([0, 0], [3, 2])
    ok, msg = check_path(P, [0, 0], [3, 2])
    if not ok:
        return f"A_star_planner([0, 0], [3, 2]) on an empty grid: {msg}"
    if len(P) != 6:
        return f"the shortest path from [0, 0] to [3, 2] has 5 steps (6 cells), got {len(P)} cells"


def _check_trajectory():
    load_grid(empty_grid())
    T = path_to_trajectory([[25, 25], [26, 25], [27, 25]], "distance")
    expected = [[0.0, 0.005, 0.005, 0.0], [0.0833, 0.015, 0.005, 0.0], [0.1667, 0.025, 0.005, 0.0]]
    if T is None or len(T) != 3:
        return f"should return 3 poses, got {T}"
    for p, q in zip(T, expected):
        if len(p) != 4 or not all(_close(a, b, 1e-3) for a, b in zip(p, q)):
            return f"expected (rounded) {expected}, got {[[round(v, 4) for v in p] for p in T]}"


def _check_traj_tracker():
    T = [[0.1 * k, 0.01 * k, 0.0, 0.0] for k in range(101)]       # straight line along +x
    out = trajectory_tracking_controller(T, [0.0, 0.0, 0.0], 0.0)
    if out is None:
        return "returned None (did you copy point_tracking_controller from Lab 3?)"
    phi_l, phi_r = out
    if not (phi_l > 0 and phi_r > 0 and _close(phi_l, phi_r, 1e-6)):
        return (f"robot at the start of a straight trajectory along +x should get two equal "
                f"positive wheel speeds, got {phi_l:.3f}, {phi_r:.3f}")


def run_self_checks():
    checks = [("Node", _check_node), ("get_step_cost", _check_step_cost),
              ("get_heuristic", _check_heuristic), ("get_neighbors", _check_neighbors),
              ("is_goal", _check_is_goal), ("build_path", _check_build_path),
              ("A_star_planner (small)", _check_a_star_small),
              ("path_to_trajectory", _check_trajectory),
              ("trajectory_tracking_controller", _check_traj_tracker)]
    passed = sum(_check(name, fn) for name, fn in checks)
    print(f"  {passed} out of {len(checks)} self-checks passed")


if __name__ == "__main__":
    print("=== Self-checks (one function at a time) ===")
    run_self_checks()
    print("\n=== Part 2: A* (shortest path) ===")
    run_a_star_unit_tests()
    print("\n=== Part 3: A* (minimum time) ===")
    run_time_cost_unit_tests()
    print("\n=== Part 4: trajectory tracking ===")
    run_traj_tracker_unit_tests()
    # print("\n=== Part 5: density analysis ===")   # slow: run it once for your report
    # run_density_analysis()
