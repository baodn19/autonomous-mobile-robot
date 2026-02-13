"""
AI
Prompt:
You said
Write for me a file that allow the differential robot to drive straight and rotate in place. Below is the description of the functions:

straight(distance, speed)
- degree = distance/ PERIMETER x 360
- run_from_degree(degree, speed)
- Same for the other motor

rotate(degree, direction, speed) # direction = 1: right; direction = 0: left
- If direction = 1: set speed to negative; otherwise keep the same positive speed

Usage: It gave me a structure that I followed. I still had to rewrite 90% of the code. Only the import and the try: except: from main() is kept.
"""

import math
import time
from robot_systems.robot import HamBot

# Initialize the robot with default settings (Unit: mm)
WHEEL_DIAMETER = 90
WHEEL_PERIMETER = WHEEL_DIAMETER * math.pi
AXLE_TRACK = 184  

def straight(bot, distance, speed):
    """
    - Function: Drives the robot straight.
    - Arguments:
        bot: the robot object
        distance (mm): the distance the robot is going to travel
        speed (rpm): angular velocity for both motor
    """
    # Calculate the # of wheel rotation
    rotation = (distance / WHEEL_PERIMETER)
    print(f"Driving Straight: {distance}mm | Wheel Rotations: {rotation:.1f}")
    
    bot.run_motors_for_rotations(rotation, left_speed=speed, right_speed=speed)


def rotateInPlace(bot, degree, direction, speed):
    """
    - Function: Rotates the robot in place.
    - Arguments:
        bot: the robot object
        degree: The angle for the ROBOT to turn in comparison to EAST (positive x-axis).
        direction: 1 = Right, -1 = Left.
        speed (rpm): angular velocity for both motor
    """
    # Calculate the # of wheel rotation
    rotation = degree / 360 * AXLE_TRACK * math.pi / WHEEL_PERIMETER
    print(f"Rotating {'Right' if direction == 1 else 'Left'} {degree}Â° | Wheel Rotations: {rotation:.1f}")

    # Direction 1 (Right): Left motor forward (-), Right motor backward (-)
    # Direction -1 (Left): Left motor backward (+), Right motor forward (+)
    speed *= direction
    bot.run_motors_for_rotations(rotation, left_speed=speed, right_speed=-speed)
    
def circle(bot, radius, direction, outer_speed):
    """
    - Function: Drives the robot in a circle.
    - Arguments:
        bot: the robot object
        radius (mm): the distance from instantaneous center of curvature (ICC) to the robot
        direction: 1 = Right, -1 = Left.
        outer_speed (rpm): angular velocity of the outer motor
    """
    # Calculation
    angular_velocity = WHEEL_DIAMETER / 2 * outer_speed / (radius + AXLE_TRACK / 2)
    inner_speed = angular_velocity * (radius -  AXLE_TRACK / 2) / WHEEL_DIAMETER * 2
    outer_rotation = (2 * radius + AXLE_TRACK) * math.pi / WHEEL_PERIMETER
    inner_rotation = (2 * radius - AXLE_TRACK) * math.pi / WHEEL_PERIMETER
    print({inner_speed}, {outer_speed}, {angular_velocity})
    print({inner_rotation}, {outer_rotation})
    
    if direction == 1: # Right
        bot.run_left_motor_for_rotations(inner_rotation, inner_speed, False)
        bot.run_right_motor_for_rotations(outer_rotation, outer_speed, True)
    else: # Left
        bot.run_right_motor_for_rotations(inner_rotation, inner_speed, False)
        bot.run_left_motor_for_rotations(outer_rotation, outer_speed, True)
        
def rectangle(bot, length, width, speed):
    """
    - Function: Drives the robot CW in a rectangle. The robot starts at the middle of the width on the left side.
    - Arguments:
        bot: the robot object
        length (mm): the length of rectangle
        width (mm): the width of rectangle
        speed (rpm): angular velocity for both motor
    """
    straight(bot, width / 2, speed)
    time.sleep(1)
        
    rotateInPlace(bot, 90, 1, speed)
    time.sleep(1)
        
    straight(bot, length, speed)
    time.sleep(1)
    
    rotateInPlace(bot, 90, 1, speed)
    time.sleep(1)
    
    straight(bot, width, speed)
    time.sleep(1)
    
    rotateInPlace(bot, 90, 1, speed)
    time.sleep(1)
    
    straight(bot, length, speed)
    time.sleep(1)
    
    rotateInPlace(bot, 90, 1, speed)
    time.sleep(1)
    
    straight(bot, width / 2, speed)
    time.sleep(1)


# --- Test Routine ---
if __name__ == "__main__":
    try:
        my_robot = HamBot(lidar_enabled=False, camera_enabled=False)
        
        rectangle(1000, 2000, 50)
        time.sleep(1)
        
        circle(my_robot, 500, 1, 50)
        time.sleep(1)
        
        circle(my_robot, 1000, -1, 50)
        time.sleep(1)
        
        rectangle(1500, 500, 50)
        time.sleep(1)
        
        circle(my_robot, 750, 1, 50)
        time.sleep(1)
        
        circle(my_robot, 250, -1, 50)
        time.sleep(1)
        

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        if 'my_robot' in locals():
            my_robot.disconnect_robot()
