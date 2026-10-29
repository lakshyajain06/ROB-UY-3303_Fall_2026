"""
ROB-UY 3303 - Lab 04: Tree Search Motion Planning - the map
You EDIT this file: add your own maps to MAPS and choose ACTIVE_MAP.
lab_4.py and gui_4.py import it.

THE GRID
  The robot plans in a grid of 200 x 200 cells, each cell 1 cm x 1 cm,
  so the grid covers the 2 m x 2 m free space of the room corner of Lab 3.
  Cell [0, 0] touches the room corner (-0.25, -0.25) m, so the robot start
  pose (0, 0, 0) is in cell [25, 25].

        y ^
          |   cell [i, j]:  i = column (along x), j = row (along y)
   wall 2 |
          |  R ->  x          R = robot start pose (0, 0, 0)
          +------------------ wall 1
   corner = (-0.25, -0.25)

  A map is a 200 x 200 array of 1s and 0s: grid[i][j] = 1 if cell [i, j]
  holds an obstacle, 0 if it is free. MAP below is the array of ACTIVE_MAP.

THE MAPS
  You describe a map with boxes instead of typing 40000 numbers.
    "boxes":       obstacles, (x_min, y_min, x_max, y_max) in meters.
    "slow_zones":  regions where the robot must drive slowly (Part 3,
                   minimum time planning), (x_min, y_min, x_max, y_max, k)
                   in meters, where k in (0, 1] is the traversability:
                   the robot may drive at k * ROBOT_SPEED_M_S in that region.
    "lidar_sees_boxes": True if the boxes are taller than the LIDAR. Then the
                   particle filter (and the simulated LIDAR) use the box sides
                   as walls. Use False for boxes lower than the LIDAR.

  REAL OBSTACLES: measure the position of the box with a tape from the two
  walls, exactly like the robot start pose in Lab 3. A box whose side is
  d_back cm from the back wall (wall 2) and d_side cm from the side wall
  (wall 1) has x_min = d_back/100 - 0.25 and y_min = d_side/100 - 0.25.
  Leave at least 30 cm between a box and a wall (or another box) if you want
  the robot to be able to drive between them.

  SLOW ZONES on the real robot: mark the region on the floor with tape. The
  robot does not feel it, but your trajectory (Part 4) makes it drive slowly
  there, as if the floor were sand, carpet or a crowded area.
"""
import numpy as np

# =====================================================================
# PROVIDED - GRID DEFINITION (do not change)
# =====================================================================
GRID_N = 200                        # cells along x and along y
CELL_SIZE_M = 0.01                  # m, one cell is 1 cm x 1 cm
GRID_ORIGIN_M = (-0.25, -0.25)      # m, corner of cell [0, 0] = room corner
ROBOT_RADIUS_M = 0.10               # m, same as the GUI
C_SPACE_MARGIN_M = 0.05             # m, extra room for localization and tracking error

# =====================================================================
# STUDENT EDIT - YOUR MAPS
# =====================================================================
MAPS = {
    # no obstacles: only the walls
    "empty": {
        "boxes": [],
        "slow_zones": [],
        "lidar_sees_boxes": True,
    },
    # Part 2, simulation: one box between the start pose and the goal (1.5, 0.0)
    "box_sim": {
        "boxes": [(0.60, -0.15, 0.90, 0.15)],
        "slow_zones": [],
        "lidar_sees_boxes": True,
    },
    # Part 2, robot: a REAL box (about 30 cm x 20 cm, taller than the LIDAR)
    # centred at (0.75, 0.00) m, i.e. its back side is 85 cm from the back
    # wall and its lower side is 15 cm from the side wall.
    # Drive from (0, 0) to (1.5, 0.0) around it.
    "box_robot": {
        "boxes": [(0.60, -0.10, 0.90, 0.10)],
        "slow_zones": [],
        "lidar_sees_boxes": True,
    },
    # Part 3, simulation: a slow zone (k = 0.20) across the straight route
    # from (0, 0) to (1.5, 0.0), and a box above it.
    "terrain_sim": {
        "boxes": [(0.60, 0.90, 0.90, 1.20)],
        "slow_zones": [(0.50, -0.25, 1.00, 0.50, 0.20)],
        "lidar_sees_boxes": True,
    },
    # Part 3, robot: the same slow zone, marked with tape on the floor.
    "terrain_robot": {
        "boxes": [],
        "slow_zones": [(0.50, -0.25, 1.00, 0.50, 0.20)],
        "lidar_sees_boxes": True,
    },
    # STUDENT TODO: add your own maps here (2 for simulation, 2 for the robot).
}

ACTIVE_MAP = "box_sim"              # map loaded when the GUI starts


# =====================================================================
# PROVIDED - MAP BUILDING (do not change)
# =====================================================================
def _cell_range(v_min, v_max, origin):
    """Cells [first, last+1) overlapping the interval [v_min, v_max] (m)."""
    first = int(np.floor((v_min - origin) / CELL_SIZE_M + 1e-9))
    last = int(np.ceil((v_max - origin) / CELL_SIZE_M - 1e-9))
    return max(first, 0), min(last, GRID_N)


def _fill_box(grid, x_min, y_min, x_max, y_max, value):
    i0, i1 = _cell_range(x_min, x_max, GRID_ORIGIN_M[0])
    j0, j1 = _cell_range(y_min, y_max, GRID_ORIGIN_M[1])
    if i0 < i1 and j0 < j1:
        grid[i0:i1, j0:j1] = value


def build_obstacle_grid(name):
    """200 x 200 array, grid[i][j] = 1 if cell [i, j] holds an obstacle."""
    grid = np.zeros((GRID_N, GRID_N), dtype=np.uint8)
    for box in MAPS[name]["boxes"]:
        _fill_box(grid, *box, 1)
    return grid


def build_traversability_grid(name):
    """200 x 200 array of traversability k in (0, 1] (1 = normal floor)."""
    grid = np.ones((GRID_N, GRID_N), dtype=float)
    for x_min, y_min, x_max, y_max, k in MAPS[name]["slow_zones"]:
        if not 0.0 < k <= 1.0:
            raise ValueError(f"traversability must be in (0, 1], got {k}")
        _fill_box(grid, x_min, y_min, x_max, y_max, k)
    return grid


def grow_obstacles(grid, radius_m):
    """Configuration space of a round robot (Lecture 08A: plan for a point
    robot by "growing" the obstacles). Cell [i, j] is 1 if a robot centred
    there would be closer than radius_m to an obstacle cell or to the edge
    of the grid (the walls)."""
    r = int(np.ceil(radius_m / CELL_SIZE_M))
    padded = np.ones((GRID_N + 2 * r, GRID_N + 2 * r), dtype=bool)
    padded[r:r + GRID_N, r:r + GRID_N] = grid.astype(bool)
    out = np.zeros((GRID_N, GRID_N), dtype=bool)
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj <= r * r:
                out |= padded[r + di:r + di + GRID_N, r + dj:r + dj + GRID_N]
    return out.astype(np.uint8)


def build_c_space(name):
    """Obstacle grid grown by the robot radius + margin."""
    return grow_obstacles(build_obstacle_grid(name), ROBOT_RADIUS_M + C_SPACE_MARGIN_M)


def box_walls(name):
    """Sides of the boxes as wall segments ((x1, y1), (x2, y2)), for the
    LIDAR model. Empty if the LIDAR cannot see the boxes."""
    if not MAPS[name].get("lidar_sees_boxes", True):
        return []
    walls = []
    for x_min, y_min, x_max, y_max in MAPS[name]["boxes"]:
        walls += [((x_min, y_min), (x_max, y_min)), ((x_max, y_min), (x_max, y_max)),
                  ((x_max, y_max), (x_min, y_max)), ((x_min, y_max), (x_min, y_min))]
    return walls


def random_map(density, seed=None, min_box_cells=5, max_box_cells=20):
    """Random obstacle grid (for the analysis of Part 5): square boxes of
    min_box_cells to max_box_cells cells are added at random until the
    fraction of occupied cells reaches density (0 to 1)."""
    rng = np.random.default_rng(seed)
    grid = np.zeros((GRID_N, GRID_N), dtype=np.uint8)
    target = density * GRID_N * GRID_N
    while grid.sum() < target:
        s = int(rng.integers(min_box_cells, max_box_cells + 1))
        i, j = int(rng.integers(0, GRID_N - s + 1)), int(rng.integers(0, GRID_N - s + 1))
        grid[i:i + s, j:j + s] = 1
    return grid


MAP = build_obstacle_grid(ACTIVE_MAP)           # the 200 x 200 array of 1s and 0s
TRAVERSABILITY = build_traversability_grid(ACTIVE_MAP)
