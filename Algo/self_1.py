import heapq
import json
import math
import time
from itertools import permutations # <--- ADD THIS LINE
from queue import PriorityQueue
from typing import List, Tuple, Optional

# ==============================================================================
# 1. CONSTANTS AND PARAMETERS
# ==============================================================================

# Grid parameters
GRID_SIZE = 40  # 40x40 grid, 1 cell = 5cm
GRID_UNITS = 20  # 20x20 units, 1 unit = 10cm
STEP_ANGLE = 45  # 45-degree angle step
STEP_SIZE = 0.5  # 5cm movement step (1 cell)
MAX_ITERATIONS = 100000  # Safety limit for A* iterations

# Constants for robot dimensions (in 10cm/unit scale)
ROBOT_LENGTH = 2.3  # 23cm (Front-to-Back)
ROBOT_WIDTH = 1.8  # 18cm (Side-to-Side)

# Photo Constraints (BL wheel + 25cm target clearance)
REQUIRED_BL_WHEEL_DIST_TO_FACE = 4.8  # 2.3 (L) + 2.5 (C)
REQUIRED_BL_WHEEL_DIST_TO_CENTER = 5.3  # 4.8 + 0.5 (obstacle half-width)

# Grid Safety Buffers (Minimal, as footprint check handles safety)
ROBOT_CLEARANCE_RADIUS = 2  # 10cm buffer (2 cells) - CRITICAL FIX
EDGE_BUFFER = 2  # 10cm buffer from the edge - CRITICAL FIX
EDGE_SAFETY_MARGIN = 1.0  # 1.0 units (10cm)


# ==============================================================================
# 2. UTILITY & SAFETY FUNCTIONS
# ==============================================================================

def get_robot_corners(x: float, y: float, theta: float) -> List[Tuple[float, float]]:
    """Calculates the four corners of the robot's footprint. (x, y) is the bottom-left wheel."""
    theta_rad = math.radians(theta)

    corners_rel = [
        (0, 0),  # BL (Bottom-Left)
        (ROBOT_LENGTH, 0),  # BR (Bottom-Right)
        (ROBOT_LENGTH, ROBOT_WIDTH),  # TR (Top-Right)
        (0, ROBOT_WIDTH)  # TL (Top-Left)
    ]

    corners_abs = []
    for rel_x, rel_y in corners_rel:
        # 1. Rotate the relative point
        rot_x = rel_x * math.cos(theta_rad) - rel_y * math.sin(theta_rad)
        rot_y = rel_x * math.sin(theta_rad) + rel_y * math.cos(theta_rad)

        # 2. Translate by the wheel position
        abs_x = x + rot_x
        abs_y = y + rot_y
        corners_abs.append((abs_x, abs_y))

    return corners_abs


def is_valid_footprint(x: float, y: float, theta: float, grid: List[List[bool]]) -> bool:
    """Checks if the entire robot footprint is in valid cells."""

    points_to_check = get_robot_corners(x, y, theta)

    for corner_x, corner_y in points_to_check:
        # Convert continuous coordinate (0-20) to 40x40 grid index (0-39)
        grid_x = int(math.floor(corner_x * 2))
        grid_y = int(math.floor(corner_y * 2))

        # 1. Check Grid Bounds
        if not (0 <= grid_x < GRID_SIZE and 0 <= grid_y < GRID_SIZE):
            return False

        # 2. Check Static Grid Validity
        if not grid[grid_y][grid_x]:
            return False

    return True


def get_discrete_node(x: float, y: float, theta: float) -> Tuple[int, int, int]:
    """Converts continuous state (x, y, theta) to a discrete, hashable state (40x40 grid)."""
    grid_x = int(round(x * 2))
    grid_y = int(round(y * 2))
    discrete_theta = int(round(theta / STEP_ANGLE) * STEP_ANGLE % 360)

    grid_x = max(0, min(grid_x, GRID_SIZE - 1))
    grid_y = max(0, min(grid_y, GRID_SIZE - 1))

    return (grid_x, grid_y, discrete_theta)


def get_movement_cost(action: str) -> float:
    """Assigns cost based on movement type."""
    if 'forward' in action or 'backward' in action:
        return 1.0
    else:
        return 1.2  # Penalty for turning while moving


def move(x: float, y: float, theta: float, action: str) -> Tuple[float, float, float]:
    """Applies a kinematic movement step (BL Wheel reference)."""
    theta_rad = math.radians(theta)
    new_theta = theta
    new_x, new_y = x, y

    # Movement calculation based on BL Wheel as reference
    if 'forward' in action or 'backward' in action:
        dist = STEP_SIZE
        if 'backward' in action:
            dist = -STEP_SIZE

        new_x += dist * math.cos(theta_rad)
        new_y += dist * math.sin(theta_rad)

    if 'left' in action:
        new_theta = (theta + STEP_ANGLE) % 360
    elif 'right' in action:
        new_theta = (theta - STEP_ANGLE + 360) % 360

    return new_x, new_y, new_theta


def heuristic_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance heuristic for A*."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# ==============================================================================
# 3. GRID & PHOTO POINT SETUP
# ==============================================================================

def mark_invalid_cells(grid: List[List[bool]], obstacles: List[Tuple[int, int, str]]):
    """Marks cells within a MINIMAL_CLEARANCE_RADIUS (10cm) of obstacles and edges."""

    for obs_x, obs_y, _ in obstacles:
        obs_center_x = (obs_x + 0.5) * 2
        obs_center_y = (obs_y + 0.5) * 2

        for i in range(-ROBOT_CLEARANCE_RADIUS, ROBOT_CLEARANCE_RADIUS + 1):
            for j in range(-ROBOT_CLEARANCE_RADIUS, ROBOT_CLEARANCE_RADIUS + 1):
                new_x = int(obs_center_x + i)
                new_y = int(obs_center_y + j)

                if 0 <= new_x < GRID_SIZE and 0 <= new_y < GRID_SIZE:
                    if math.sqrt(i ** 2 + j ** 2) <= ROBOT_CLEARANCE_RADIUS:
                        grid[new_y][new_x] = False

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if i < EDGE_BUFFER or i >= (GRID_SIZE - EDGE_BUFFER) or \
                    j < EDGE_BUFFER or j >= (GRID_SIZE - EDGE_BUFFER):
                grid[j][i] = False


def get_photo_point(obstacle_x: int, obstacle_y: int, obstacle_direction: str) -> List[float]:
    """Generates the BL Wheel coordinate for a photo, ensuring 4.8 unit clearance from the face."""

    obs_center_x = obstacle_x + 0.5
    obs_center_y = obstacle_y + 0.5

    direction_map = {'U': 90, 'D': 270, 'L': 180, 'R': 0}

    # CRITICAL FIX: Safely determine the obstacle direction (int)
    if obstacle_direction in direction_map:
        direction = direction_map[obstacle_direction]
    else:
        # If it's not a char ('U', 'D', etc.), assume it's a numeric string (e.g., '180')
        try:
            direction = int(obstacle_direction)
        except ValueError:
            # Fallback for unexpected string input
            direction = 0

            # Angle is the robot's heading (BL to BR axis)

    if direction == 0:  # Obstacle on East wall, robot faces West
        photo_x = obs_center_x + REQUIRED_BL_WHEEL_DIST_TO_CENTER
        photo_y = obs_center_y
        angle = 180

    elif direction == 180:  # Obstacle on West wall, robot faces East
        photo_x = obs_center_x - REQUIRED_BL_WHEEL_DIST_TO_CENTER
        photo_y = obs_center_y
        angle = 0

    elif direction == 90:  # Obstacle on North wall, robot faces South
        photo_x = obs_center_x
        photo_y = obs_center_y - REQUIRED_BL_WHEEL_DIST_TO_CENTER
        angle = 270

    elif direction == 270:  # Obstacle on South wall, robot faces North
        photo_x = obs_center_x
        photo_y = obs_center_y + REQUIRED_BL_WHEEL_DIST_TO_CENTER
        angle = 90
    else:
        # Default/Error case
        photo_x, photo_y, angle = obs_center_x, obs_center_y, 0

    # Clip coordinates to bounds (safety measure)
    GRID_UNITS = 20
    photo_x = max(0.0, min(photo_x, GRID_UNITS))
    photo_y = max(0.0, min(photo_y, GRID_UNITS))

    return [photo_x, photo_y, angle]


def get_all_points(global_obstacles: List[Tuple[int, int, str]], grid: List[List[bool]]) -> List[List[float]]:
    """Generates all photo points, validating their footprint immediately."""

    # Start point moved to 4.0, 4.0 for guaranteed safety margin (CRITICAL FIX)
    points = [[4.0, 4.0, 90]]
    valid_obstacles = []

    for obstacle in global_obstacles:
        print(f"DEBUG: Processing obstacle: {obstacle}")
        x, y, direction = obstacle
        photo_point = get_photo_point(x, y, direction)

        if is_valid_footprint(photo_point[0], photo_point[1], photo_point[2], grid):
            points.append(photo_point)
            valid_obstacles.append(obstacle)
            print(f"DEBUG: Added valid photo point: {photo_point}")
        else:
            # Debug output mimicking the original failure state
            grid_x = int(math.floor(photo_point[0] * 2))
            grid_y = int(math.floor(photo_point[1] * 2))
            print(f"DEBUG: SKIPPED invalid photo point: {photo_point} for obstacle: {obstacle}")
            print(f"DEBUG: Photo point maps to grid position ({grid_x}, {grid_y})")
            print(
                f"DEBUG: Grid bounds check: 0 <= {grid_x} < {GRID_SIZE} and 0 <= {grid_y} < {GRID_SIZE} = {0 <= grid_x < GRID_SIZE and 0 <= grid_y < GRID_SIZE}")
            if 0 <= grid_x < GRID_SIZE and 0 <= grid_y < GRID_SIZE:
                print(f"DEBUG: Grid cell value: {grid[grid_y][grid_x]}")
                print(
                    f"DEBUG: Edge buffer check: x < {EDGE_BUFFER} or x > {GRID_SIZE - EDGE_BUFFER} or y < {EDGE_BUFFER} or y > {GRID_SIZE - EDGE_BUFFER} = {grid_x < EDGE_BUFFER or grid_x > (GRID_SIZE - EDGE_BUFFER) or grid_y < EDGE_BUFFER or grid_y > (GRID_SIZE - EDGE_BUFFER)}")

    print(f"DEBUG: Total valid obstacles: {len(valid_obstacles)} out of {len(global_obstacles)}")
    return points


# ==============================================================================
# 4. PATHFINDING: A* SEARCH
# ==============================================================================

def reconstruct_path(backtrack_path: dict, backtrack_move: dict, end_node: Tuple[int, int, int]) -> Tuple[
    List[List[float]], List[str]]:
    """Reconstructs the continuous path and movement list from the discrete nodes."""
    path_discrete = []
    moves = []
    current_node = end_node
    while current_node is not None:
        path_discrete.append(current_node)
        moves.append(backtrack_move.get(current_node))
        current_node = backtrack_path.get(current_node)

    path_discrete.reverse()
    moves.reverse()

    continuous_path = []
    for disc_x, disc_y, theta in path_discrete:
        continuous_path.append([disc_x / 2.0, disc_y / 2.0, float(theta)])

    return continuous_path, moves[1:]


def a_star_search(start: List[float], goal: List[float], grid: List[List[bool]]) -> Optional[
    Tuple[List[List[float]], List[str], float, None, None]]:
    """A* search for path planning."""

    start_discrete_node = get_discrete_node(start[0], start[1], start[2])

    if not is_valid_footprint(start[0], start[1], start[2], grid) or \
            not is_valid_footprint(goal[0], goal[1], goal[2], grid):
        print("DEBUG A*: ERROR - Start or Goal position is invalid!")
        return None

    open_list = PriorityQueue()
    cost = {}
    backtrack_path = {}
    backtrack_move = {}

    open_list.put((0, start_discrete_node))
    cost[start_discrete_node] = 0
    backtrack_path[start_discrete_node] = None
    backtrack_move[start_discrete_node] = None

    iterations = 0
    actions = ['forward', 'backward', 'left', 'right']

    while not open_list.empty() and iterations < MAX_ITERATIONS:
        iterations += 1
        _, current_discrete_node = open_list.get()

        disc_x, disc_y, theta = current_discrete_node
        current_x, current_y = disc_x / 2.0, disc_y / 2.0

        # Goal Check: Check proximity and angle tolerance (loose tolerance for speed)
        if heuristic_distance(current_x, current_y, goal[0], goal[1]) < STEP_SIZE * 2 and abs(
                theta - goal[2]) < STEP_ANGLE * 2:
            path, moves = reconstruct_path(backtrack_path, backtrack_move, current_discrete_node)
            total_cost = cost[current_discrete_node]
            print(f"DEBUG A*: SUCCESS - Found path. Final position: ({current_x:.1f}, {current_y:.1f})")
            return path, moves, total_cost, None, None

        for action in actions:
            new_x_cont, new_y_cont, new_theta = move(current_x, current_y, theta, action)
            new_discrete_node = get_discrete_node(new_x_cont, new_y_cont, new_theta)

            if not is_valid_footprint(new_x_cont, new_y_cont, new_theta, grid):
                continue

            new_cost = cost[current_discrete_node] + get_movement_cost(action)

            if new_discrete_node not in cost or new_cost < cost[new_discrete_node]:
                cost[new_discrete_node] = new_cost
                priority = new_cost + heuristic_distance(new_x_cont, new_y_cont, goal[0], goal[1])
                open_list.put((priority, new_discrete_node))
                backtrack_path[new_discrete_node] = current_discrete_node
                backtrack_move[new_discrete_node] = action

    if iterations >= MAX_ITERATIONS:
        print(f"DEBUG A*: FAILED - Max iterations reached ({MAX_ITERATIONS}).")
    else:
        print("DEBUG A*: FAILED - No path found.")
    return None


# ==============================================================================
# 5. PATH PLANNING: TSP & TRAVERSAL
# ==============================================================================

def calculate_distance_matrix(points: List[List[float]], grid: List[List[bool]]) -> Tuple[
    List[List[float]], List[List[float]]]:
    """Calculates the distance (cost) matrix between all points using A*."""
    num_points = len(points)
    cost_matrix = [[float('inf')] * num_points for _ in range(num_points)]
    path_details = [[None] * num_points for _ in range(num_points)]  # Store full details

    for i in range(num_points):
        for j in range(num_points):
            if i == j:
                cost_matrix[i][j] = 0
                continue

            start = points[i]
            goal = points[j]
            print(f"DEBUG A*: Starting search from {start} to {goal}")

            result = a_star_search(start, goal, grid)

            if result:
                path, moves, cost = result
                cost_matrix[i][j] = cost
                path_details[i][j] = {'path': path, 'moves': moves, 'cost': cost}

    # Print matrix rows to match required debug format
    for i, row in enumerate(cost_matrix):
        print(row)

    return cost_matrix, path_details


def find_shortest_hamiltonian_path(distance_matrix: List[List[float]]) -> Optional[Tuple[List[int], float]]:
    """Finds the shortest path starting at point 0 and visiting all others (TSP)."""
    num_points = len(distance_matrix)
    if num_points <= 1:
        return [0] if num_points == 1 else None, 0.0

    min_distance = float('inf')
    best_path = None

    for path_tuple in permutations(range(1, num_points)):
        current_path = [0] + list(path_tuple)
        current_distance = 0
        is_valid_path = True

        for i in range(num_points - 1):
            start_index = current_path[i]
            end_index = current_path[i + 1]
            distance = distance_matrix[start_index][end_index]

            if distance == float('inf'):
                is_valid_path = False
                break

            current_distance += distance

        if is_valid_path and current_distance < min_distance:
            min_distance = current_distance
            best_path = current_path

    return best_path, min_distance


# --- Algo_Front.py (Replace get_path_from_obstacles and all lines after it) ---

def get_path_and_movements(obstacles):
    """
    Main function to calculate the full path, movements, and cost.

    Returns:
        tuple: (list of (path_coords, moves), total_cost, final_coordinates)
        or None if no path is found.
    """
    print(f"DEBUG: Processing {len(obstacles)} obstacles: {obstacles}")

    grid = [[True for _ in range(40)] for _ in range(40)]
    mark_invalid_cells(grid, obstacles)
    print("DEBUG: Grid marked with invalid cells")

    # 1. Get and Filter Photo Points
    points = get_all_points(obstacles, grid)
    print(f"DEBUG: Generated {len(points)} points: {points}")

    if len(points) < 2:
        print("ERROR: No valid obstacles found - all photo positions are invalid")
        return None

    # 2. Calculate Distance Matrix (A* Run)
    num_points = len(points)
    cost_matrix = [[0 for _ in range(num_points)] for _ in range(num_points)]
    path_details = [[None] * num_points for _ in range(num_points)]

    for i in range(num_points):
        for j in range(i + 1, num_points):
            path, moves, cost, _, _ = a_star_search(points[i], points[j], grid)
            if path:
                cost_matrix[i][j] = cost
                cost_matrix[j][i] = cost
                path_details[i][j] = path_details[j][i] = {'path': path, 'moves': moves, 'cost': cost}
            else:
                cost_matrix[i][j] = float('inf')
                cost_matrix[j][i] = float('inf')

    # 3. Filter Unreachable Points (CRITICAL FIX)
    valid_indices = [0]
    for j in range(1, len(points)):
        if cost_matrix[0][j] != float('inf'):
            valid_indices.append(j)

    if len(valid_indices) < len(points):
        print(f"WARNING: Filtering {len(points) - len(valid_indices)} points unreachable from the start.")

        filtered_points = [points[i] for i in valid_indices]
        filtered_cost_matrix = [[cost_matrix[old_i][old_j] for old_j in valid_indices] for old_i in valid_indices]
        filtered_path_details = [[path_details[old_i][old_j] for old_j in valid_indices] for old_i in valid_indices]

        points = filtered_points
        cost_matrix = filtered_cost_matrix
        path_details = filtered_path_details

    for i, row in enumerate(cost_matrix):
        print(f"DEBUG: Distance matrix row {i}: {row}")

    # 4. Find Shortest Hamiltonian Path (TSP)
    best_path, min_distance = find_shortest_hamiltonian_path(cost_matrix)
    print(f"DEBUG: Best path found: {best_path}, min_distance: {min_distance}")

    if best_path is None:
        print("Error: No valid Hamiltonian path found after filtering.")
        return None

    # 5. Assemble Movements
    full_movements = []
    total_cost = 0.0
    final_coordinates = []

    for i in range(len(best_path) - 1):
        start_index = best_path[i]
        end_index = best_path[i + 1]

        details = path_details[start_index][end_index]

        # Structure matches what grid.py animation loop expects: (path_coords, moves)
        full_movements.append((details['path'], details['moves']))
        total_cost += details['cost']
        final_coordinates.extend(details['path'])

    # Returns the list of segments, total cost, and all coordinates
    return full_movements, total_cost, final_coordinates

# -----------------------------------------------------------------
