import math
import time
from robot_systems.robot import HamBot

# Initialize the robot with default settings (LIDAR and Camera disabled for this task)
robot = HamBot(lidar_enabled=False, camera_enabled=False)


WHEEL_DIAMETER = 90  # mm
PERIMETER = WHEEL_DIAMETER * math.pi
AXLE_TRACK = 184   # mm

def straight(bot, distance, speed):
    """
    Moves the robot straight.
    Logic: degree = distance / PERIMETER * 360
    """
    # 1. Calculate the degrees the wheels need to turn
    degrees = (distance / PERIMETER) * 360
    
    print(f"Driving Straight: {distance}mm | Wheel Degrees: {degrees:.1f}°")
    
    # 2. Convert to rotations for the HamBot wrapper
    bot.left_motor.run_for_degrees(-degrees, speed)
    bot.right_motor.run_for_degrees(degrees, speed)


def rotateInPlace(bot, degree, direction, speed):
    """
    Rotates the robot in place.
    Args:
        degree: The angle for the ROBOT to turn.
        direction: 1 = Right, 0 = Left.
        speed: Positive motor speed.
    """
    print(f"Rotating {'Right' if direction == 1 else 'Left'} {degree}° | Wheel Degrees: {degree:.1f}°")

    # 3. Determine motor speeds based on direction
    # Direction 1 (Right): Left motor forward (-), Right motor backward (-)
    # Direction 0 (Left): Left motor backward (+), Right motor forward (+)
    if direction == 1:
        speed = -speed

    # 4. Run motors
    # Note: HamBot's run_motors_for_rotations handles the synchronization (blocking/non-blocking)
    bot.left_motor.run_for_degrees(-degree, speed)
    bot.right_motor.run_for_degrees(degree, speed)


# --- Test Routine ---
if __name__ == "__main__":
    try:
        my_robot = HamBot()
        
        print("--- Starting Routine ---")
        
        # Drive forward 20cm
        straight(my_robot, distance=50, speed=25)
        time.sleep(0.5)
        
        # Rotate Right 90 degrees (Direction 1)
        rotateInPlace(my_robot, degree=90, direction=1, speed=25)
        time.sleep(0.5)
        
        # Rotate Left 90 degrees (Direction 0)
        rotateInPlace(my_robot, degree=90, direction=0, speed=25)
        time.sleep(0.5)
        
        print("--- Routine Complete ---")

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        if 'my_robot' in locals():
            my_robot.disconnect_robot()