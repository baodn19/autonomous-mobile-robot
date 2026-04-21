"""
Author: Ngoc Bao Dinh
UID: U27463715

AI usage 
Prompt 1:
Goal: Program a Python script to fulfill the task 2 outline for physical robots in the pdf "Lab 4 Code Submission.pdf".
For physical robots information, refer to "RobotHardware2026.pdf".
For the functions that can be implement in the Python script, refer to "robot.py"
For sensor functions, refer to the other python scripts
"""

import time
import random
import sys
from robot_systems.robot import HamBot

WHEEL_CIRCUMFERENCE_MM = 90 * 3.14159
TRACK_WIDTH_MM = 184
CELL_SIZE_MM = 600
ROTATIONS_FORWARD = CELL_SIZE_MM / WHEEL_CIRCUMFERENCE_MM
ROTATIONS_90DEG = ((TRACK_WIDTH_MM * 3.14159) / 4) / WHEEL_CIRCUMFERENCE_MM
WALL_THRESHOLD_MM = 450

"""
    Function: Builds the 4x4 grid map for Maze 2 with outer and inner wall definitions
    Returns: Dictionary representing the maze map
"""
def build_maze2_map_4x4():
    maze = {i: {'N': 0, 'E': 0, 'S': 0, 'W': 0} for i in range(1, 17)}
    
    # Outer boundaries
    for i in range(1, 17):
        if i <= 4: maze[i]['N'] = 1
        if i >= 13: maze[i]['S'] = 1
        if i % 4 == 1: maze[i]['W'] = 1
        if i % 4 == 0: maze[i]['E'] = 1

    # Inner walls for Maze 2 (S-shape configuration)
    inner_walls = [
        # Top horizontal
        (2, 'S', 6, 'N'), (3, 'S', 7, 'N'),
        # Left vertical
        (5, 'E', 6, 'W'),
        # Middle horizontal
        (6, 'S', 10, 'N'), (7, 'S', 11, 'N'), (8, 'S', 12, 'N'),
        # Right vertical
        (11, 'E', 12, 'W'),
        # Bottom horizontal
        (10, 'S', 14, 'N'), (11, 'S', 15, 'N')
    ]
    for c1, dir1, c2, dir2 in inner_walls:
        maze[c1][dir1] = 1
        maze[c2][dir2] = 1
        
    return maze


"""
    Function: Averages lidar readings around a target angle to reduce noise
    Parameter:
    - scan: list of 360 lidar distance readings
    - target_angle: angle in degrees to center the averaging around (0-359)
    - window: number of degrees on either side of target_angle to include in the average (default=2 for a 5-degree window) 
    Returns: average distance in mm, or float('inf') if no valid readings are found 
"""
def get_average_distance(scan, target_angle, window=2):
    """Averages lidar readings around a target angle to reduce noise."""
    valid_readings = [
        scan[(target_angle + i) % 360] 
        for i in range(-window, window + 1) 
        if scan[(target_angle + i) % 360] > 0
    ]
    return sum(valid_readings) / len(valid_readings) if valid_readings else float('inf')

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    maze = build_maze2_map_4x4()
    
    # Initialize 250 particles evenly distributed across 16 cells
    particles = [{'cell': random.randint(1, 16), 'orientation': random.choice(['N', 'E', 'S', 'W'])} for _ in range(250)]
    
    time.sleep(2) # Sensor warmup
    
    try:
        while True:
            scan = bot.get_range_image()
            heading = bot.get_heading(blocking=True)
            
            if scan == -1 or heading is None:
                time.sleep(0.1)
                continue

            discrete_heading = round(heading / 90.0) * 90 % 360
            
            dist_F = get_average_distance(scan, 180)
            dist_B = get_average_distance(scan, 0)
            dist_L = get_average_distance(scan, 90)
            dist_R = get_average_distance(scan, 270)
            
            # Align Lidar data to absolute compass directions
            obs = {}
            if discrete_heading == 90:   # Facing North
                obs['N'] = 1 if dist_F < WALL_THRESHOLD_MM else 0
                obs['E'] = 1 if dist_R < WALL_THRESHOLD_MM else 0
                obs['S'] = 1 if dist_B < WALL_THRESHOLD_MM else 0
                obs['W'] = 1 if dist_L < WALL_THRESHOLD_MM else 0
            elif discrete_heading == 0:  # Facing East
                obs['N'] = 1 if dist_L < WALL_THRESHOLD_MM else 0
                obs['E'] = 1 if dist_F < WALL_THRESHOLD_MM else 0
                obs['S'] = 1 if dist_R < WALL_THRESHOLD_MM else 0
                obs['W'] = 1 if dist_B < WALL_THRESHOLD_MM else 0
            elif discrete_heading == 270: # Facing South
                obs['N'] = 1 if dist_B < WALL_THRESHOLD_MM else 0
                obs['E'] = 1 if dist_L < WALL_THRESHOLD_MM else 0
                obs['S'] = 1 if dist_F < WALL_THRESHOLD_MM else 0
                obs['W'] = 1 if dist_R < WALL_THRESHOLD_MM else 0
            elif discrete_heading == 180: # Facing West
                obs['N'] = 1 if dist_R < WALL_THRESHOLD_MM else 0
                obs['E'] = 1 if dist_B < WALL_THRESHOLD_MM else 0
                obs['S'] = 1 if dist_L < WALL_THRESHOLD_MM else 0
                obs['W'] = 1 if dist_F < WALL_THRESHOLD_MM else 0

            # 1. Correction 
            weights = []
            for p in particles:
                w = 1.0
                cell = p['cell']
                for d in ['N', 'E', 'S', 'W']:
                    z = obs[d]
                    s = maze[cell][d]
                    if z == 0 and s == 0: w *= 0.6
                    elif z == 1 and s == 0: w *= 0.4
                    elif z == 1 and s == 1: w *= 0.8
                    elif z == 0 and s == 1: w *= 0.2
                weights.append(w)

            # Normalize weights
            total_w = sum(weights)
            if total_w == 0:
                weights = [1.0 / 250] * 250
            else:
                weights = [w / total_w for w in weights]

            # 2. Resampling
            indices = random.choices(range(250), weights=weights, k=250)
            particles = [
                {
                    'cell': particles[i]['cell'], 
                    'orientation': random.choice(['N', 'E', 'S', 'W'])
                } 
                for i in indices
            ]

            # Output Generation
            counts = {i: 0 for i in range(1, 17)}
            for p in particles:
                counts[p['cell']] += 1

            mode_cell = max(counts, key=counts.get)
            max_count = counts[mode_cell]
            percent = max_count / 250.0

            print("\nParticle Distribution:")
            for row in range(4):
                print(" ".join([f"{counts[row * 4 + col + 1]:4d}" for col in range(4)]))
            print(f"Mode Cell: {mode_cell} ({percent * 100:.1f}%)")

            if percent >= 0.8:
                print("Localization successful (>= 80% particles in one cell).")
                break

            # 3. Prediction 
            if dist_F > WALL_THRESHOLD_MM:
                bot.run_motors_for_rotations(ROTATIONS_FORWARD, left_speed=50, right_speed=50)
                for p in particles:
                    c = p['cell']
                    d = p['orientation']
                    if maze[c][d] == 0:
                        if d == 'N': p['cell'] -= 4
                        elif d == 'S': p['cell'] += 4
                        elif d == 'E': p['cell'] += 1
                        elif d == 'W': p['cell'] -= 1
            else:
                bot.run_motors_for_rotations(ROTATIONS_90DEG, left_speed=50, right_speed=-50)
            
            time.sleep(1) 

    except KeyboardInterrupt:
        pass
    finally:
        bot.disconnect_robot()

if __name__ == "__main__":
    main()