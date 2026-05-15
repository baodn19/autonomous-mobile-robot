"""
Author: Ngoc Bao Dinh
UID: U27463715

AI usage 
Prompt 1:
Goal: Program a Python script to fulfill the task 1 outline for physical robots in the pdf "Lab 5 Code Submission.pdf".
For physical robots information, refer to "RobotHardware2026.pdf".
For the functions that can be implement in the Python script, refer to "robot.py"
For sensor functions, refer to the other python scripts
"""

import time
import heapq
from robot_systems.robot import HamBot

# --- Kinematic Constants ---
ROTATIONS_PER_CELL = 2.122  # 60 cm travel
ROTATIONS_90_DEG = 0.511    # 90-degree zero-radius turn
BASE_SPEED = 50             # RPM

# --- Map Representation ---
# Data structure encoding adjacency constraints (walls) from physical Maze 1.
# Format: (x, y): [(neighbor_x, neighbor_y), ...]
# (0,0) is bottom-left (Green), (3,3) is top-right (Blue).
maze_graph = {
    # Row 0 (Bottom)
    (0, 0): [(1, 0), (0, 1)],
    (1, 0): [(0, 0), (2, 0)],
    (2, 0): [(1, 0), (3, 0)],
    (3, 0): [(2, 0), (3, 1)],
    
    # Row 1
    (0, 1): [(0, 0), (1, 1), (0, 2)],
    (1, 1): [(0, 1), (2, 1)],
    (2, 1): [(1, 1)], 
    (3, 1): [(3, 0)], # Dead end
    
    # Row 2
    (0, 2): [(0, 1), (0, 3)],
    (1, 2): [(2, 2)], # Dead end
    (2, 2): [(1, 2), (3, 2)],
    (3, 2): [(2, 2), (3, 3)],
    
    # Row 3 (Top)
    (0, 3): [(0, 2), (1, 3)],
    (1, 3): [(0, 3), (2, 3)],
    (2, 3): [(1, 3), (3, 3)],
    (3, 3): [(2, 3), (3, 2)]
}

def get_neighbors(cell):
    """Returns valid neighboring cells avoiding walls."""
    return maze_graph.get(cell, [])

def plan_path_dijkstra(start, goal):
    """
    Computes the shortest path using Dijkstra's Algorithm with unit edge costs.
    Returns an ordered list of cells from start to goal.
    """
    # Priority queue stores tuples of (cumulative_cost, current_cell, path)
    queue = [(0, start, [start])]
    visited = set()
    
    while queue:
        cost, curr, path = heapq.heappop(queue)
        
        if curr == goal:
            return path
            
        if curr in visited:
            continue
            
        visited.add(curr)
        
        for neighbor in get_neighbors(curr):
            if neighbor not in visited:
                # Add 1 to the cost for each move (unit edge cost)
                heapq.heappush(queue, (cost + 1, neighbor, path + [neighbor]))
                
    return None

def align_to_walls(bot, tolerance_mm=15, max_attempts=5):
    """
    Corrects heading drift by squaring the robot against the nearest wall.
    Uses a proportional approach on symmetric Lidar angles.
    """
    for _ in range(max_attempts):
        scan = bot.get_range_image()
        if scan == -1 or not scan:
            return

        # Check front wall presence (180 degrees)
        front_dist = scan[180]
        
        # If a wall is within 0.5 m, use it for alignment
        if 0 < front_dist < 500:
            left_probe = scan[165]
            right_probe = scan[195]
            
            # Ensure probes are hitting a valid surface
            if left_probe > 0 and right_probe > 0:
                diff = left_probe - right_probe
                
                # If the difference is within tolerance, alignment is complete
                if abs(diff) <= tolerance_mm:
                    break
                    
                # Proportional adjustment: if left_probe > right_probe, angled right. Turn left.
                adjust_speed = 10 if diff > 0 else -10
                bot.run_motors_for_rotations(0.02, left_speed=-adjust_speed, right_speed=adjust_speed)
                time.sleep(0.1) # Allow Lidar to stabilize

def execute_path(bot, path):
    """Drives the robot along the computed cell-by-cell path with Lidar correction."""
    current_heading = 90  # Assuming robot starts facing North (Y-axis positive)

    if bot.lidar is None:
        print("Error: Lidar must be enabled for physical execution alignment.")
        return

    for i in range(1, len(path)):
        curr = path[i-1]
        nxt = path[i]

        # Determine required heading for next cell (4-connected moves)
        dx = nxt[0] - curr[0]
        dy = nxt[1] - curr[1]
        
        if dx == 1: target_heading = 0      # East
        elif dx == -1: target_heading = 180 # West
        elif dy == 1: target_heading = 90   # North
        elif dy == -1: target_heading = 270 # South
        else: target_heading = current_heading

        # Turn to target heading
        turn_robot(bot, current_heading, target_heading)
        current_heading = target_heading
        
        align_to_walls(bot)

        # Move forward one cell
        bot.run_motors_for_rotations(ROTATIONS_PER_CELL, left_speed=BASE_SPEED, right_speed=BASE_SPEED)
        time.sleep(0.2) 
        
        align_to_walls(bot)

def turn_robot(bot, current_heading, target_heading):
    """Executes a zero-radius turn to reach the target heading using dead reckoning."""
    diff = (target_heading - current_heading) % 360
    
    if diff == 0:
        return
    elif diff == 90:
        # Turn Left (Counter-Clockwise)
        bot.run_motors_for_rotations(ROTATIONS_90_DEG, left_speed=-BASE_SPEED, right_speed=BASE_SPEED)
    elif diff == 270:
        # Turn Right (Clockwise)
        bot.run_motors_for_rotations(ROTATIONS_90_DEG, left_speed=BASE_SPEED, right_speed=-BASE_SPEED)
    elif diff == 180:
        # U-Turn
        bot.run_motors_for_rotations(ROTATIONS_90_DEG * 2, left_speed=BASE_SPEED, right_speed=-BASE_SPEED)
        
    time.sleep(0.5)

def main():
    # Hardcoded Start/Goal for physical execution testing
    start_cell = (3, 1) # Updated to match robot location in image
    goal_cell = (0, 0)  # Example goal
    
    # 1. Plan Path using Dijkstra
    path = plan_path_dijkstra(start_cell, goal_cell)
    
    if not path:
        print("Error: No valid path found.")
        return

    # 2. Console Output Verification
    path_str = " -> ".join([f"({r},{c})" for r, c in path])
    print(f"Path: [{path_str}]")
    print(f"Path length: {len(path) - 1}")

    # 3. Hardware Execution
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    
    try:
        time.sleep(2) # Give hardware time to settle
        execute_path(bot, path)
    except KeyboardInterrupt:
        print("\nExecution interrupted by user.")
    finally:
        bot.disconnect_robot()

if __name__ == "__main__":
    main()
