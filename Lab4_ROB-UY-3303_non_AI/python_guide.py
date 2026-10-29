"""
ROB-UY 3303 - Lab 04: PYTHON GUIDE (for the no-AI section)

Every Python idea you need for lab_4.py, with small examples you can run:

    python python_guide.py

Read a section, run the file, look at what it prints, then change the
examples and run it again. That is the fastest way to learn Python.

The examples are NOT about A*: they show the Python tools, and you decide
how to use them in your lab.

CONTENTS
   1. Variables, numbers and printing
   2. Lists
   3. Lists of lists: cells and poses
   4. if / elif / else, and / or / not
   5. for loops, range, continue
   6. while loops
   7. Functions: inputs, return, several outputs, default inputs
   8. None
   9. Classes and objects
  10. Objects that point to other objects (chains)
  11. The math module
  12. numpy 2D arrays (the grid)
  13. Using other files: import, module.name
  14. Timing code and random numbers
  15. Means (averages)
  16. Reading error messages
  17. Debugging tips
"""
import math
import random
import time

import numpy as np


def section(title):
    print("\n" + "=" * 70 + "\n" + title + "\n" + "=" * 70)


# =====================================================================
section("1. VARIABLES, NUMBERS AND PRINTING")
# =====================================================================
# A variable is a name for a value. No type to declare.
speed = 0.12            # a float (decimal number)
cells = 200             # an int (whole number)
name = "box_sim"        # a string (text)
done = False            # a bool: True or False

print(speed, cells, name, done)

# Arithmetic: + - * /   ** (power)   // (whole division)   % (remainder)
print("0.01 / 0.12 =", 0.01 / 0.12)
print("7 // 2 =", 7 // 2, "   7 % 2 =", 7 % 2, "   3 ** 2 =", 3 ** 2)

# Change a variable using its old value:
count = 0
count = count + 1       # same as  count += 1
print("count =", count)

# f-strings: put values inside text with {}. :.3f = 3 decimals.
time_s = 0.01 / 0.12
print(f"time = {time_s:.3f} s for {cells} cells")

# Careful: = stores a value, == compares two values.
print("speed == 0.12 ->", speed == 0.12)

# Comparing decimals: computers round floats, so 0.1 + 0.2 is NOT exactly 0.3.
print("0.1 + 0.2 == 0.3 ->", 0.1 + 0.2 == 0.3)
print("close enough   ->", abs((0.1 + 0.2) - 0.3) < 1e-9)


# =====================================================================
section("2. LISTS")
# =====================================================================
stations = ["A", "B", "C"]          # a list: values in order
empty = []                          # an empty list

print("first:", stations[0])        # indices start at 0
print("last :", stations[-1])       # -1 = last, -2 = the one before
print("length:", len(stations))

stations.append("D")                # add at the end
print("after append:", stations)

removed = stations.pop(1)           # remove element number 1 and get it back
print("popped", removed, "->", stations)

# Slices: a[start:stop] gives a copy from start up to (NOT including) stop
numbers = [10, 20, 30, 40, 50]
print("numbers[1:3] =", numbers[1:3])       # [20, 30]
print("numbers[1:]  =", numbers[1:])        # everything except the first
print("numbers[::-1] =", numbers[::-1])     # a reversed copy

# Joining two lists makes a new list
print("[1, 2] + [3] =", [1, 2] + [3])

# Is a value in a list?
print("'C' in stations ->", "C" in stations)

# Lists can be compared with ==
print("[1, 2] == [1, 2] ->", [1, 2] == [1, 2])


# =====================================================================
section("3. LISTS OF LISTS: CELLS AND POSES")
# =====================================================================
# In the lab, a cell is [i, j] and a pose is [x, y, theta]. A path is a
# LIST of cells, a trajectory is a LIST of [t, x, y, theta].
cell = [25, 25]
i = cell[0]
j = cell[1]
print("cell", cell, "-> i =", i, " j =", j)

# "Unpacking": give a name to each element in one line
i, j = cell
x, y, theta = [0.5, 0.2, 1.57]
print("unpacked:", i, j, "|", x, y, theta)

# A list of lists
route = [[0, 0], [1, 0], [1, 1]]
print("second cell of the route:", route[1])
print("its j:", route[1][1])

# Part of a list: drop the first element of a pose with time
p = [3.0, 0.5, 0.2, 1.57]           # [t, x, y, theta]
print("p[0] (time) =", p[0], "   p[1:] (no time) =", p[1:])

# Building a NEW cell from an old one (the old one is not changed)
moved = [cell[0] + 1, cell[1]]
print("one step in +x:", moved, " original still:", cell)

# Careful: b = a does NOT copy a list, both names are the SAME list
a = [1, 2]
b = a
b.append(3)
print("a after changing b:", a, "<- surprise! use list(a) or a.copy() to copy")


# =====================================================================
section("4. if / elif / else, and / or / not")
# =====================================================================
k = 0.2
if k == 1.0:
    print("normal floor")
elif k > 0.5:
    print("a bit slow")
else:
    print("very slow")

# Combine conditions
i, j = 210, 50
if i < 0 or i >= 200 or j < 0 or j >= 200:
    print(f"[{i}, {j}] is off a 200 x 200 grid")
if not (0 <= i < 200):              # Python allows  0 <= i < 200
    print("same test, written with not")

# Indentation (the spaces at the start of a line) decides what is inside
# the if. Use 4 spaces. Mixing tabs and spaces gives an IndentationError.


# =====================================================================
section("5. for LOOPS, range, continue")
# =====================================================================
for k in range(4):                  # k = 0, 1, 2, 3   (4 is NOT included)
    print("k =", k)

for k in range(2, 5):               # k = 2, 3, 4
    print("from 2:", k)

for s in ["A", "B", "C"]:           # loop over the elements of a list
    print("station", s)

route = [[0, 0], [1, 0], [1, 1]]
for k in range(len(route)):         # loop over the indices of a list
    print("index", k, "cell", route[k])

# Loop over pairs of values, and unpack them
for di, dj in [(1, 0), (0, 1)]:
    print("move by", di, dj)

# continue: skip the rest of this turn of the loop, go to the next one
for n in range(6):
    if n % 2 == 1:
        continue                    # skip odd numbers
    print("even:", n)

# Build a list in a loop
squares = []
for n in range(5):
    squares.append(n * n)
print("squares:", squares)

# Repeat something 100 times when you do not need the counter: use _
total = 0
for _ in range(100):
    total = total + 1
print("repeated", total, "times")


# =====================================================================
section("6. while LOOPS")
# =====================================================================
# A while loop repeats as long as the condition is True.
n = 1
while n < 100:
    n = n * 2
print("first power of 2 above 100:", n)

# Walking along a list until a condition is met. Check the INDEX first,
# so you never read past the end of the list:
temperatures = [12, 15, 19, 24, 30]
k = 0
while k < len(temperatures) - 1 and temperatures[k] < 20:
    k = k + 1
print("first temperature >= 20:", temperatures[k], "at index", k)

# If nothing matches, k stops at the LAST index (no crash):
k = 0
while k < len(temperatures) - 1 and temperatures[k] < 100:
    k = k + 1
print("nothing >= 100, so k stops at the last index:", k)

# A while loop that never becomes False runs forever: stop the program
# with Ctrl + C in the terminal.


# =====================================================================
section("7. FUNCTIONS")
# =====================================================================
def distance(p, q):
    """Straight line distance between two points p = [x, y], q = [x, y]."""
    return math.sqrt((q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2)


print("distance([0, 0], [3, 4]) =", distance([0, 0], [3, 4]))


# Several outputs: return them separated by commas, unpack them
def min_and_max(values):
    return min(values), max(values)


lo, hi = min_and_max([4, 9, 1])
print("min", lo, "max", hi)


# Default input: used when the caller does not give it
def travel_time(d, mode="walk"):
    if mode == "run":
        return d / 3.0
    return d / 1.0


print("walk:", travel_time(6), "  run:", travel_time(6, "run"))

# A function without "return" returns None (see section 8). If a check
# prints "got None", the function did not return anything yet.


def forgot_return(a, b):
    a + b                           # computed, but never returned!


print("forgot_return(1, 2) =", forgot_return(1, 2))


# =====================================================================
section("8. None")
# =====================================================================
# None means "nothing / no value". Test it with "is None".
answer = None
if answer is None:
    print("no answer yet")
answer = 42
if answer is not None:
    print("answer:", answer)

# Typical use: a function returns None when it found nothing.


def find(values, target):
    for v in values:
        if v == target:
            return v
    return None


print("find 3:", find([1, 2, 3], 3), "  find 7:", find([1, 2, 3], 7))


# =====================================================================
section("9. CLASSES AND OBJECTS")
# =====================================================================
# A class is a template; an object is one thing made from it. __init__
# runs when the object is created, and "self" is the object itself.
class Room:
    def __init__(self, name, width, length):
        self.name = name            # member variables: self.something
        self.width = width
        self.length = length
        self.area = width * length  # can be computed from the others


kitchen = Room("kitchen", 3.0, 4.0)
bedroom = Room("bedroom", 3.5, 3.0)
print(kitchen.name, kitchen.area, "|", bedroom.name, bedroom.area)

# Change a member variable
kitchen.width = 5.0
print("new width:", kitchen.width, " (area was computed once:", kitchen.area, ")")

# A list of objects
rooms = [kitchen, bedroom]
for r in rooms:
    print("room", r.name)

# The object with the smallest value of a member variable
smallest = rooms[0]
for r in rooms:
    if r.area < smallest.area:
        smallest = r
print("smallest room:", smallest.name)


# =====================================================================
section("10. OBJECTS THAT POINT TO OTHER OBJECTS (CHAINS)")
# =====================================================================
# A member variable can hold ANOTHER object. Here each station remembers
# the station before it on a train line; the first one has None.
class Station:
    def __init__(self, name, previous):
        self.name = name
        self.previous = previous


first = Station("Coney Island", None)
second = Station("Bay Pkwy", first)
third = Station("Atlantic Av", second)

print("third.previous.name =", third.previous.name)
print("third.previous.previous.name =", third.previous.previous.name)

# Walk back along the chain until there is no previous station:
s = third
names = []
while s is not None:
    names.append(s.name)
    s = s.previous                  # move one step back
print("from the end to the start:", names)
print("from the start to the end:", names[::-1])

# "is" checks if two names are the SAME object
print("second.previous is first ->", second.previous is first)


# =====================================================================
section("11. THE math MODULE")
# =====================================================================
print("math.sqrt(16)      =", math.sqrt(16))
print("math.hypot(3, 4)   =", math.hypot(3, 4), "  (= sqrt(3^2 + 4^2))")
print("math.atan2(1, 1)   =", math.atan2(1, 1), "rad  (angle of the vector (dx=1, dy=1))")
print("math.degrees(pi/2) =", math.degrees(math.pi / 2))
print("math.radians(90)   =", math.radians(90))
print("math.cos(0), sin(0) =", math.cos(0), math.sin(0))
print("math.floor(2.7)    =", math.floor(2.7))
print("math.inf > 1e100   ->", math.inf > 1e100, " (infinity is bigger than any number)")
print("abs(-3)            =", abs(-3))
print("min(4, 2), max(4, 2) =", min(4, 2), max(4, 2))
# Careful: atan2 takes (dy, dx), y FIRST.


# =====================================================================
section("12. numpy 2D ARRAYS (THE GRID)")
# =====================================================================
# The map is a 200 x 200 numpy array. Here a small 5 x 4 one.
board = np.zeros((5, 4), dtype=int)     # 5 columns (i), 4 rows (j), all 0
board[2][1] = 1                         # cell [2, 1] = 1
board[0, 3] = 1                         # same thing, other way to write it
print("board[2][1] =", board[2][1], "  board[0][0] =", board[0][0])

# Fill many cells at once with slices (the stop index is NOT included)
board[1:4, 3] = 1                       # i = 1, 2, 3 at j = 3
print("board (printed with i down, j across):\n", board)
print("number of cells equal to 1:", board.sum())

# An array full of one value
costs = np.full((5, 4), math.inf)
costs[0][0] = 0.0
print("costs[0][0] =", costs[0][0], " costs[1][1] =", costs[1][1])

# Careful: reading outside the array crashes (IndexError) ...
try:
    board[5][0]
except IndexError as e:
    print("board[5][0] ->", "IndexError:", e)
# ... and a NEGATIVE index does NOT crash: it reads from the other end!
print("board[-1][0] reads the LAST column, no error:", board[-1][0])
# So check that 0 <= i < size and 0 <= j < size BEFORE reading a cell.

# Copy an array: b = a.copy()  (b = a is the SAME array, as with lists)


# =====================================================================
section("13. USING OTHER FILES: import, module.name")
# =====================================================================
# "import map_4" runs map_4.py and gives access to its names with a dot:
#     map_4.MAPS, map_4.GRID_N, map_4.random_map(0.1, seed=0)
# Same for the libraries:  math.hypot, np.zeros, time.perf_counter
#
# Global constants (names in CAPITALS at the top of a file, like
# CELL_SIZE_M or ROBOT_SPEED_M_S) can be READ inside any function of that
# file. You do not need to pass them as inputs.
STEP_M = 0.01


def length_of(n_steps):
    return n_steps * STEP_M             # reads the global constant


print("100 steps =", length_of(100), "m")

# Editing a dictionary in map_4.py: a dict maps a name to a value.
example_maps = {
    "empty": {"boxes": []},
    "one_box": {"boxes": [(0.60, -0.15, 0.90, 0.15)]},
}
print("boxes of one_box:", example_maps["one_box"]["boxes"])
# A box is a tuple (x_min, y_min, x_max, y_max). A tuple is like a list
# in round brackets. Do not forget the commas between boxes and maps.


# =====================================================================
section("14. TIMING CODE AND RANDOM NUMBERS")
# =====================================================================
t0 = time.perf_counter()                # a clock, in seconds
total = sum(range(1_000_000))
seconds = time.perf_counter() - t0
print(f"summing 1 million numbers took {seconds * 1000:.1f} ms")

rng = np.random.default_rng(0)          # create ONCE; seed 0 = same numbers every run
print("3 random ints in [0, 200):", rng.integers(0, 200), rng.integers(0, 200), rng.integers(0, 200))
print("python's random module too:", random.randint(1, 6))


# =====================================================================
section("15. MEANS (AVERAGES)")
# =====================================================================
lengths = [1.2, 0.8, 1.5]
print("mean =", sum(lengths) / len(lengths))
print("numpy mean =", np.mean(lengths))
# Careful: the mean of an EMPTY list divides by 0 -> ZeroDivisionError.
values = []
if len(values) > 0:
    print(sum(values) / len(values))
else:
    print("no values: no mean")


# =====================================================================
section("16. READING ERROR MESSAGES")
# =====================================================================
# Python prints a "Traceback". READ THE LAST LINES FIRST: the last line
# is the error, the line above it shows the file and line number.
#
#   Traceback (most recent call last):
#     File "lab_4.py", line 312, in get_neighbors
#       if is_occupied(X_new):
#   IndexError: index 200 is out of bounds for axis 0 with size 200
#
# Common errors and what they usually mean:
#
#   IndexError: list index out of range / index 200 is out of bounds
#       You read element number k of a list or array that is too short.
#       Check the size BEFORE reading.
#   TypeError: 'NoneType' object is not subscriptable / not iterable
#       A variable is None and you used it like a list: a function you
#       called has no "return" yet, or returned None.
#   TypeError: unsupported operand type(s) for +: 'float' and 'NoneType'
#       Same: a function returned None instead of a number.
#   AttributeError: 'Node' object has no attribute 'g_value'
#       The member variable was never set in __init__ (check its spelling,
#       and that you wrote self.g_value, not just g_value).
#   NameError: name 'X_new' is not defined
#       A typo, or the variable is created later / inside another function.
#   IndentationError / unexpected indent
#       The spaces at the start of the line are wrong. Use 4 spaces.
#   ZeroDivisionError
#       You divided by 0 (e.g. the mean of an empty list, or a speed of 0).
#   The program never ends
#       A while loop whose condition never becomes False. Ctrl + C stops it.
#       The traceback then shows where it was stuck.
no_path = forgot_return(1, 2)           # None, from section 7
examples = [
    ("IndexError", lambda: [1, 2, 3][3]),
    ("TypeError (None)", lambda: no_path[0]),
    ("AttributeError", lambda: Room("x", 1, 1).height),
    ("ZeroDivisionError", lambda: sum([]) / len([])),
]
for label, bad in examples:
    try:
        bad()
    except Exception as e:
        print(f"{label:18s} -> {type(e).__name__}: {e}")


# =====================================================================
section("17. DEBUGGING TIPS")
# =====================================================================
# 1. Run "python lab_4.py" after EVERY function you write, and fix the
#    first failing self-check before you go on.
# 2. Print what you have, at the place where it goes wrong:
#        print("n.X =", n.X, " g =", n.g_value, " children:", len(children))
# 3. Try your function on a TINY case you can do by hand (a 3-cell path,
#    a grid with 1 obstacle) before the 200 x 200 maps.
# 4. Print the type of a strange value:   print(type(P), P)
# 5. Inside a loop that runs thousands of times, print only a few times:
#        if count < 5: print(...)
# 6. Remove your debug prints when it works (they slow down the planner).
print("Done. Now change the examples above and run this file again.")
