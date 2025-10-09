import heapq
import json
import math
import time
from itertools import permutations
from queue import PriorityQueue
from typing import List, Tuple

import flask
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

# Grid parameters
GRID_SIZE = 40

grid = [[True for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

PHOTO_DISTANCE = 3.2


def load_obstacles_from_state_file(filename="state.json"):
    # Read the state.json file
    with open(filename, 'r') as f:
        data = json.load(f)

    # Extract only the obstacles, ignoring the robot
    obstacles_raw = data['obstacles']

    # Format the obstacles and unpack correctly as (x, y, direction)
    obstacles = [(obs[0], obs[1], obs[2]) for obs in obstacles_raw]

    return obstacles


# Example usage
# global_obstacles = load_obstacles_from_state_file()
# print("Loaded obstacles:", global_obstacles)

def mark_invalid_cells(grid, obstacles):
    # ...
    for obs_x, obs_y, _ in obstacles:
        obs_center_x = (obs_x + 0.5) * 2  # Convert to 40x40 grid (5cm per cell)
        obs_center_y = (obs_y + 0.5) * 2
        # ...
        # Mark cells within 4.2 units (approx 21cm) of the obstacle center as invalid
        for i in range(-5, 6):
            for j in range(-5, 6):
                new_x = int(obs_center_x + i)
                new_y = int(obs_center_y + j)
                if 0 <= new_x < 40 and 0 <= new_y < 40:
                    # **CRITICAL CHANGE: Check this line**
                    if math.sqrt(i ** 2 + j ** 2) <= 4.2:  # Was 5.2
                        grid[new_y][new_x] = False

    # Mark cells within 2 units (10cm) of the edges as invalid
    for i in range(40):
        for j in range(40):
            # **CRITICAL CHANGE: Check this line**
            if i < 2 or i > 37 or j < 2 or j > 37:  # Was i < 3 or i > 36 or j < 3 or j > 36
                grid[j][i] = False


def print_grid(grid, obstacles):
    """
    Print the grid in square format, marking obstacles and invalid areas.
    Obstacles: "O", Invalid areas: "X", Valid areas: "0".
    """
    for i in range(GRID_SIZE - 1, -1, -1):
        row = []
        for j in range(GRID_SIZE):
            if (i, j) in obstacles:
                row.append("O")
            elif not grid[i][j]:
                row.append("X")
            else:
                row.append("0")
        print(" ".join(row))


def get_photo_point(obstacle_x, obstacle_y, obstacle_direction):
    """
    Generate a photo point based on the obstacle position and direction.
    """
    if obstacle_direction == 180 or obstacle_direction == "W" or obstacle_direction == "L":
        photo_x = obstacle_x - PHOTO_DISTANCE
        photo_y = obstacle_y + 0.5
        angle = 0  # Camera facing right
    elif obstacle_direction == 0 or obstacle_direction == "E" or obstacle_direction == "R":
        photo_x = obstacle_x + PHOTO_DISTANCE
        photo_y = obstacle_y + 0.5
        angle = 180  # Camera facing left
    elif obstacle_direction == 90 or obstacle_direction == "N" or obstacle_direction == "U":
        photo_x = obstacle_x + 0.5
        photo_y = obstacle_y + PHOTO_DISTANCE
        angle = 270  # Camera facing down
    elif obstacle_direction == 270 or obstacle_direction == "S" or obstacle_direction == "D":
        photo_x = obstacle_x + 0.5
        photo_y = obstacle_y - PHOTO_DISTANCE
        angle = 90  # Camera facing up
    else:
        raise ValueError("Invalid obstacle direction")
    return [photo_x, photo_y, angle]


RADIUS = 25
STEP_ANGLE = 90  # <<< CRITICAL CHANGE: Enforcing 90-degree pivots
STEP_SIZE = 1


def move_forward(x, y, theta, step=STEP_SIZE):
    """Move forward."""
    theta_rad = math.radians(theta)
    new_x = x + step * math.cos(theta_rad)
    new_y = y + step * math.sin(theta_rad)
    return new_x, new_y, theta


def move_backward(x, y, theta, step=STEP_SIZE):
    """Move backward."""
    theta_rad = math.radians(theta)
    new_x = x - step * math.cos(theta_rad)
    new_y = y - step * math.sin(theta_rad)
    return new_x, new_y, theta


# REMOVED: turn_left_forward, turn_left_backward, turn_right_forward, turn_right_backward
# REMOVED: LF/LB/RF/RB_TURN_DISPLACEMENT_ANGLE/DISPLACEMENT constants

def turn_left_90(x, y, theta):
    """Turn left by 90 degrees in place."""
    return x, y, (theta + 90) % 360


def turn_right_90(x, y, theta):
    """Turn right by 90 degrees in place."""
    return x, y, (theta - 90) % 360


# Since the previous code had print statements after the now-removed functions,
# I will keep the existing print output but simplify it to just the new pure turns
# to show functionality (though the original purpose was for the complex moves).
# I'll comment out the prints to avoid confusion since the complex turns are gone.
# print(turn_left_forward(0, 0, 90)) # REMOVED
# print(turn_left_backward(0, 0, 90)) # REMOVED
# print(turn_right_forward(0, 0, 90)) # REMOVED
# print(turn_right_backward(0, 0, 90)) # REMOVED


def heuristic_distance(x1, y1, x2, y2):
    # diag = min(abs(x1 - x2), abs(y1 - y2))
    # return diag + abs(x1 - x2) + abs(y1 - y2)
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def heuristic_angle(theta1, theta2):
    return abs(theta1 - theta2)


def is_valid(x, y, grid):
    """
    Checks if the position (x, y) is valid in a 40x40 grid.
    Converts from 20x20 grid to 40x40 grid (each original grid cell represents 10cm, each new cell represents 5cm).
    """
    # Convert the coordinates from 20x20 scale to 40x40 scale
    grid_x = int(math.floor(x * 2))  # x*2 converts from 10cm per cell to 5cm per cell
    grid_y = int(math.floor(y * 2))

    # Check bounds and return the validity based on the grid
    if 0 <= grid_x < 40 and 0 <= grid_y < 40:
        is_valid_cell = grid[grid_y][grid_x] != False
        # Debug: Print invalid positions occasionally
        if not is_valid_cell and (grid_x + grid_y) % 10 == 0:
            print(f"DEBUG: Invalid cell at ({x:.2f}, {y:.2f}) -> grid({grid_x}, {grid_y})")
        return is_valid_cell
    return False


cost_straight = 1
cost_turn = 10  # Cost for a pure 90-degree pivot turn


def get_movement_cost(action):
    if action in ['forward', 'backward']:
        return cost_straight
    # Only pure 90-degree pivot turns allowed
    elif action in ['turn_left_90', 'turn_right_90']:
        return cost_turn
    else:
        return float('inf')


def valid_photo_point(x, y, goal):
    front_distance = 0.7
    back_distance = 1.0
    side_distance = 0.5

    print(f"DEBUG valid_photo_point: Checking position ({x:.2f}, {y:.2f}) against goal {goal}")
    print(f"DEBUG: Tolerances - front: {front_distance}, back: {back_distance}, side: {side_distance}")

    if goal[2] == 0:
        x_valid = goal[0] - back_distance <= x <= goal[0] + front_distance
        y_valid = goal[1] - side_distance <= y <= goal[1] + side_distance
        print(
            f"DEBUG: Direction 0 (East) - X range: [{goal[0] - back_distance:.2f}, {goal[0] + front_distance:.2f}], Y range: [{goal[1] - side_distance:.2f}, {goal[1] + side_distance:.2f}]")
        print(f"DEBUG: X valid: {x_valid} (x={x:.2f}), Y valid: {y_valid} (y={y:.2f})")
        if x_valid and y_valid:
            print("DEBUG: Position is VALID for photo point")
            return True
        print("DEBUG: Position is INVALID for photo point")
        return False
    elif goal[2] == 90:
        y_valid = goal[1] - back_distance <= y <= goal[1] + front_distance
        x_valid = goal[0] - side_distance <= x <= goal[0] + side_distance
        print(
            f"DEBUG: Direction 90 (North) - Y range: [{goal[1] - back_distance:.2f}, {goal[1] + front_distance:.2f}], X range: [{goal[0] - side_distance:.2f}, {goal[0] + side_distance:.2f}]")
        print(f"DEBUG: Y valid: {y_valid} (y={y:.2f}), X valid: {x_valid} (x={x:.2f})")
        if y_valid and x_valid:
            print("DEBUG: Position is VALID for photo point")
            return True
        print("DEBUG: Position is INVALID for photo point")
        return False
    elif goal[2] == 180:
        x_valid = goal[0] - front_distance <= x <= goal[0] + back_distance
        y_valid = goal[1] - side_distance <= y <= goal[1] + side_distance
        print(
            f"DEBUG: Direction 180 (West) - X range: [{goal[0] - front_distance:.2f}, {goal[0] + back_distance:.2f}], Y range: [{goal[1] - side_distance:.2f}, {goal[1] + side_distance:.2f}]")
        print(f"DEBUG: X valid: {x_valid} (x={x:.2f}), Y valid: {y_valid} (y={y:.2f})")
        if x_valid and y_valid:
            print("DEBUG: Position is VALID for photo point")
            return True
        print("DEBUG: Position is INVALID for photo point")
        return False
    elif goal[2] == 270:
        y_valid = goal[1] - front_distance <= y <= goal[1] + back_distance
        x_valid = goal[0] - side_distance <= x <= goal[0] + side_distance
        print(
            f"DEBUG: Direction 270 (South) - Y range: [{goal[1] - front_distance:.2f}, {goal[1] + back_distance:.2f}], X range: [{goal[0] - side_distance:.2f}, {goal[0] + side_distance:.2f}]")
        print(f"DEBUG: Y valid: {y_valid} (y={y:.2f}), X valid: {x_valid} (x={x:.2f})")
        if y_valid and x_valid:
            print("DEBUG: Position is VALID for photo point")
            return True
        print("DEBUG: Position is INVALID for photo point")
        return False
    print("DEBUG: Invalid goal direction")
    return False


# Function to discretize the continuous state (MUST BE ADDED)
def discretize_state(x, y, theta, precision_xy=2, precision_theta=90):  # precision_theta now set to 90
    """Rounds the continuous state to a fixed precision for A* state comparison."""
    # Round x and y to a certain decimal precision (e.g., 2 decimal places = 1 cm precision)
    x_disc = round(x, precision_xy)
    y_disc = round(y, precision_xy)

    # Normalize theta to [0, 360) and round it to the nearest STEP_ANGLE (90 degrees)
    theta_norm = theta % 360
    # Round to the nearest 90-degree increment
    theta_disc = round(theta_norm / precision_theta) * precision_theta
    theta_disc = theta_disc % 360  # Ensure it wraps around after rounding

    # Return the discretized (x, y, theta) tuple
    return (x_disc, y_disc, theta_disc)


def a_star_search(start, goal, grid):
    print(f"DEBUG A*: Starting search from {start} to {goal}")

    # Check if start and goal positions are valid
    if not is_valid(start[0], start[1], grid):
        print(f"DEBUG A*: ERROR - Start position {start} is invalid!")
        return None, None, None, None, None

    if not is_valid(goal[0], goal[1], grid):
        print(f"DEBUG A*: ERROR - Goal position {goal} is invalid!")
        return None, None, None, None, None

    open_list = PriorityQueue()
    cost = {}
    backtrack_path = {}  # Stores path nodes (discretized keys)
    backtrack_move = {}  # Stores movement steps (discretized keys)

    # Use the discretized state for tracking in the dictionary
    start_node_disc = discretize_state(start[0], start[1], start[2])

    # The start node is added to the open list and state dictionaries
    open_list.put((0, start_node_disc))
    cost[start_node_disc] = 0
    backtrack_path[start_node_disc] = None
    backtrack_move[start_node_disc] = None

    iterations = 0
    max_iterations = 100000

    # --- Available Actions (Only straight moves and 90-degree pivots) ---
    actions = [
        'forward', 'backward',
        'turn_left_90', 'turn_right_90'
    ]
    # ----------------------------------------------------------------

    while not open_list.empty() and iterations < max_iterations:
        iterations += 1
        _, current_node_disc = open_list.get()
        x, y, theta = current_node_disc  # Use the DISCRETIZED state for movement

        # Check if we've reached the goal
        # The goal check uses the discretized position but compares against the continuous goal
        if valid_photo_point(x, y, goal) and heuristic_angle(theta, goal[2]) < 15:
            total_cost = cost[current_node_disc]  # Get the total cost to reach the goal
            path, moves = reconstruct_path(backtrack_path, backtrack_move, current_node_disc)
            print(f"DEBUG A*: SUCCESS - Found path after {iterations} iterations. Final position: ({x:.2f}, {y:.2f})")
            return path, moves, total_cost, x, y  # Return the total cost and final position

        # Explore the neighboring nodes
        valid_moves = 0
        for action in actions:
            # The movement functions apply to the current state (discretized x, y, theta)
            # and produce continuous values for the new state
            new_x_cont, new_y_cont, new_theta_cont = move(x, y, theta, action)

            # --- Collision and Performance Optimization Logic ---

            # For turn-in-place actions, no collision check on x, y needed
            if action in ['turn_left_90', 'turn_right_90']:
                pass

                # For all movement actions, perform collision check
            elif not is_valid(new_x_cont, new_y_cont, grid):
                continue
            # -----------------------------------------------------------------------

            valid_moves += 1
            new_cost = cost[current_node_disc] + get_movement_cost(action)

            # Discretize the new, continuous state to use as the dictionary key
            new_node_disc = discretize_state(new_x_cont, new_y_cont, new_theta_cont)

            if new_node_disc not in cost or new_cost < cost[new_node_disc]:
                cost[new_node_disc] = new_cost

                # Heuristic Optimization
                heuristic_dist = heuristic_distance(new_x_cont, new_y_cont, goal[0], goal[1])
                heuristic_angle_weighted = 0.5 * heuristic_angle(new_theta_cont, goal[2])

                # Combine in the end
                priority = new_cost + heuristic_dist + heuristic_angle_weighted

                open_list.put((priority, new_node_disc))
                backtrack_path[new_node_disc] = current_node_disc
                backtrack_move[new_node_disc] = action

        if iterations % 50000 == 0:
            print(
                f"DEBUG A*: Iteration {iterations}, current: ({x:.2f}, {y:.2f}, {theta:.2f}), valid_moves: {valid_moves}, cost size: {len(cost)}")

    print(f"DEBUG A*: FAILED - No path found after {iterations} iterations")
    return None, None, None, None, None  # If no path is found, return None


def reconstruct_path(backtrack_path, backtrack_move, end_node):
    path = []
    moves = []
    while end_node:
        path.append(end_node)
        moves.append(backtrack_move[end_node])
        end_node = backtrack_path[end_node]
    path.reverse()
    moves.reverse()

    # Since all moves are atomic (either 1 unit straight or 90 degree pivot),
    # no further splitting is needed.
    return path, moves


def move(x, y, theta, action):
    # Only straight moves and 90-degree pivot turns are defined
    if action == 'forward':
        return move_forward(x, y, theta)
    elif action == 'backward':
        return move_backward(x, y, theta)
    elif action == 'turn_left_90':
        return turn_left_90(x, y, theta)
    elif action == 'turn_right_90':
        return turn_right_90(x, y, theta)
    # The 'turn_left'/'turn_right' actions from the original code are now invalid
    raise ValueError(f"Invalid movement action: {action}")


# --- The rest of the code is kept as is, but the final command sequence needs an update ---

def get_all_points(global_obstacles, grid=None):
    points = [[1.5, 1.5, 90]]  # Including the start point
    valid_obstacles = []

    # Create a temporary grid for validation if none provided
    if grid is None:
        temp_grid = [[True for _ in range(40)] for _ in range(40)]
        mark_invalid_cells(temp_grid, global_obstacles)
    else:
        temp_grid = grid

    for obstacle in global_obstacles:
        print(f"DEBUG: Processing obstacle: {obstacle}")
        x, y, direction = obstacle
        photo_point = get_photo_point(x, y, direction)

        # Check if the photo point is valid (within grid bounds and not in invalid area)
        if is_valid(photo_point[0], photo_point[1], temp_grid):
            points.append(photo_point)  # Generate the photo points for each obstacle
            valid_obstacles.append(obstacle)
            print(f"DEBUG: Added valid photo point: {photo_point}")
        else:
            # Debug why this photo point is invalid
            grid_x = int(math.floor(photo_point[0] * 2))
            grid_y = int(math.floor(photo_point[1] * 2))
            print(f"DEBUG: SKIPPED invalid photo point: {photo_point} for obstacle: {obstacle}")
            print(f"DEBUG: Photo point maps to grid position ({grid_x}, {grid_y})")
            print(
                f"DEBUG: Grid bounds check: 0 <= {grid_x} < 40 and 0 <= {grid_y} < 40 = {0 <= grid_x < 40 and 0 <= grid_y < 40}")
            if 0 <= grid_x < 40 and 0 <= grid_y < 40:
                print(f"DEBUG: Grid cell value: {temp_grid[grid_y][grid_x]}")
                print(
                    f"DEBUG: Edge buffer check: x < 2 or x > 37 or y < 2 or y > 37 = {grid_x < 2 or grid_x > 37 or grid_y < 2 or grid_y > 37}")
                # Check if this cell is within any obstacle's buffer zone
                for obs_x, obs_y, _ in global_obstacles:
                    obs_center_x = (obs_x + 0.5) * 2
                    obs_center_y = (obs_y + 0.5) * 2
                    distance = math.sqrt((grid_x - obs_center_x) ** 2 + (grid_y - obs_center_y) ** 2)
                    if distance <= 4.2:
                        print(f"DEBUG: Photo point is within {distance:.2f} cells of obstacle at ({obs_x}, {obs_y})")
                        break

    print(f"DEBUG: Total valid obstacles: {len(valid_obstacles)} out of {len(global_obstacles)}")
    return points


# points = get_all_points(global_obstacles)


def calculate_distance_matrix(points, grid):
    num_points = len(points)
    distance_matrix = [[0 for _ in range(num_points)] for _ in range(num_points)]

    for i in range(num_points):
        for j in range(i + 1, num_points):
            # Run A* to find the distance between points[i] and points[j]
            path, moves, total_cost, x, y = a_star_search(points[i], points[j], grid)
            if path:
                distance_matrix[i][j] = total_cost
                distance_matrix[j][i] = total_cost  # Symmetric distance
            else:
                distance_matrix[i][j] = float('inf')  # If no path found
                distance_matrix[j][i] = float('inf')
            print(distance_matrix)
    return distance_matrix


def calculate_path_distance(path, distance_matrix):
    total_distance = 0
    for i in range(len(path) - 1):
        total_distance += distance_matrix[path[i]][path[i + 1]]
    return total_distance


def find_shortest_hamiltonian_path(distance_matrix):
    num_points = len(distance_matrix)
    min_distance = float('inf')
    best_path = None

    # Try all permutations of points (excluding the start point)
    for perm in permutations(range(1, num_points)):
        path = [0] + list(perm)  # Start at point 0
        distance = calculate_path_distance(path, distance_matrix)
        if distance < min_distance:
            min_distance = distance
            best_path = path
    print(best_path)

    return best_path, min_distance


# Function to traverse the best Hamiltonian path
def traverse_best_path_with_tolerance(best_path, points, grid):
    print(f"DEBUG: Starting traversal with best_path: {best_path}")
    distances = []
    movements = []
    total_cost = 0
    current_position = points[0]  # Start at the initial position (start point)
    print(f"DEBUG: Starting position: {current_position}")

    for i in range(1, len(best_path)):
        goal_point = points[best_path[i]]
        print(f"DEBUG: Attempting to reach goal {i} (index {best_path[i]}): {goal_point}")

        # Run A* search to get the path and actual stopping point within tolerance
        path, moves, cost, x, y = a_star_search(current_position, goal_point, grid)

        if path:
            print(f"DEBUG: Successfully found path to goal {i}. Path length: {len(path)}, Cost: {cost}")
            movements.append((path, moves))  # Store the path and moves
            total_cost += cost
            current_position = path[-1]  # Use the final position in the path as the new start
            if path[-1][2] == 0:
                distances.append(goal_point[0] - x + PHOTO_DISTANCE)
            elif path[-1][2] == 90:
                distances.append(goal_point[1] - y + PHOTO_DISTANCE)
            elif path[-1][2] == 180:
                distances.append(x - goal_point[0] + PHOTO_DISTANCE)
            elif path[-1][2] == 270:
                distances.append(y - goal_point[1] + PHOTO_DISTANCE)

        else:
            print(f"DEBUG: FAILED - No valid path between {current_position} and {goal_point}")

    print(f"DEBUG: Traversal completed. Total movements: {len(movements)}, Total cost: {total_cost}")
    return movements, total_cost, distances


def plot_traversal_with_obstacles_no_moves(movements, points, global_obstacles):
    plt.figure(figsize=(8, 8))

    # Plot start and end points for reference
    start = points[0]
    plt.plot(start[0], start[1], 'go', label='Start', markersize=10)  # Start point (green)

    for i, point in enumerate(points[1:], start=1):
        plt.plot(point[0], point[1], 'ro', label=f'Point {i}', markersize=10)  # Other points (red)

    # Draw obstacles as squares
    for obstacle in global_obstacles:
        obs_x, obs_y, direction = obstacle
        # Draw the obstacle as a 1x1 square
        obstacle_patch = patches.Rectangle((obs_x, obs_y), 1, 1, edgecolor='black', facecolor='gray', lw=2)
        plt.gca().add_patch(obstacle_patch)

        # Draw orientation arrow based on the obstacle's direction
        if direction == "L":
            plt.arrow(obs_x + 0.5, obs_y + 0.5, -0.5, 0, head_width=0.3, head_length=0.2, fc='blue', ec='blue')
        elif direction == "R":
            plt.arrow(obs_x + 0.5, obs_y + 0.5, 0.5, 0, head_width=0.3, head_length=0.2, fc='blue', ec='blue')
        elif direction == "U":
            plt.arrow(obs_x + 0.5, obs_y + 0.5, 0, 0.5, head_width=0.3, head_length=0.2, fc='blue', ec='blue')
        elif direction == "D":
            plt.arrow(obs_x + 0.5, obs_y + 0.5, 0, -0.5, head_width=0.3, head_length=0.2, fc='blue', ec='blue')

    # Plot the paths (without movement annotations)
    for path, moves in movements:
        x = [pos[0] for pos in path]
        y = [pos[1] for pos in path]
        angles = [pos[2] for pos in path]

        # Plot the path
        plt.plot(x, y, 'b-', label='Path')  # Blue line for the path

        # Draw arrows for direction
        dx = np.cos(np.radians(angles))
        dy = np.sin(np.radians(angles))

        plt.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color='r', label='Direction')

    # Set plot limits
    plt.xlim(0, 20)
    plt.ylim(0, 20)

    # Add legend and title
    plt.legend()
    plt.title("Traversal of Shortest Hamiltonian Path with Obstacles (No Moves)")

    # Show the plot
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
    return 0


# After A* search is complete, print the movements
def print_movement_sequence(movements):
    print("Complete Movement Sequence:")
    for i, (path, moves) in enumerate(movements):
        print(f"Segment {i + 1}:")
        for move in moves:
            print(f"  {move}")


def change_sequence_into_commands(movements):
    movement_sequence = []

    # Simplified sequence due to only straight moves and pure 90-degree pivots

    for i in range(len(movements)):
        for j in range(len(movements[i][1])):
            move_action = movements[i][1][j]

            if move_action == 'SNAP':
                movement_sequence.append(move_action)
            elif move_action == 'turn_left_90':
                movement_sequence.append('TURN_LEFT_90')
            elif move_action == 'turn_right_90':
                movement_sequence.append('TURN_RIGHT_90')
            elif move_action is not None:
                # 'forward' or 'backward'
                movement_sequence.append(move_action.upper())

    return movement_sequence


def get_path_from_obstacles(obstacles):
    print(f"DEBUG: Processing {len(obstacles)} obstacles: {obstacles}")

    grid = [[True for _ in range(40)] for _ in range(40)]
    mark_invalid_cells(grid, obstacles)
    print("DEBUG: Grid marked with invalid cells")

    points = get_all_points(obstacles, grid)
    print(f"DEBUG: Generated {len(points)} points: {points}")

    # Check if we have any valid obstacles to visit (points should have at least 2: start + at least 1 obstacle)
    if len(points) < 2:
        print("ERROR: No valid obstacles found - all photo positions are invalid")
        return None, None

    distance_matrix = calculate_distance_matrix(points, grid)
    print(
        f"DEBUG: Distance matrix calculated: {len(distance_matrix)}x{len(distance_matrix[0]) if distance_matrix else 0}")
    for i, row in enumerate(distance_matrix):
        print(f"DEBUG: Distance matrix row {i}: {row}")

    best_path, min_distance = find_shortest_hamiltonian_path(distance_matrix)
    print(f"DEBUG: Best path found: {best_path}, min_distance: {min_distance}")

    # Check if best_path is None before proceeding
    if best_path is None:
        print("Error: No valid Hamiltonian path found.")
        return None, None

    movements, total_cost, distances = traverse_best_path_with_tolerance(best_path, points, grid)
    print(f"DEBUG: Traversal completed. Movements: {len(movements)}, Total cost: {total_cost}")

    coordinates = get_coordinates_from_movements(movements, start_x=1.5, start_y=1.5, start_theta=90)

    # print_movement_sequence(movements)
    # plot_traversal_with_obstacles_no_moves(movements, points, obstacles)
    movement_sequence = change_sequence_into_commands(movements)
    return movement_sequence, best_path, distances, coordinates


def get_coordinates_from_movements(movements, start_x=1.5, start_y=1.5, start_theta=90):
    """
    Calculate the actual coordinates for each movement starting from (start_x, start_y, start_theta).

    The 'path' element in 'movements' contains the discretized (x, y, theta) states
    that were successfully reached by the A* search. We use these states as the
    coordinates the robot is aiming for at each step.

    Returns a list of coordinates [x, y, theta] for the entire traversal.
    """
    coordinates = []

    # 1. Add the very first starting position
    current_x, current_y, current_theta = start_x, start_y, start_theta
    coordinates.append([current_x, current_y, current_theta])

    # 2. Iterate through each segment (path from one photo point to the next)
    for path, moves in movements:
        # The 'path' list from A* is a sequence of DISCRETIZED (x, y, theta) states
        # for each successful step, starting from the current_position and ending
        # at the final position within the goal tolerance.

        # The first node in 'path' is the start node, which is the previous segment's
        # final position (current_x, current_y, current_theta) already added to
        # 'coordinates' or the initial [1.5, 1.5, 90]. We only care about the *new*
        # nodes achieved by the moves.

        # The 'path' contains the full sequence of discretized states. We can append
        # all nodes *after* the initial one for this segment.
        for i in range(1, len(path)):
            # The path nodes are (x_disc, y_disc, theta_disc) tuples
            x_disc, y_disc, theta_disc = path[i]

            # Since the path nodes are the *discretized* states that were placed
            # into the A* open list/dictionary, we treat them as the achieved
            # coordinates for plotting purposes.
            coordinates.append([x_disc, y_disc, theta_disc])

            # Update the current position for the next segment's start
            current_x, current_y, current_theta = x_disc, y_disc, theta_disc

    return coordinates
