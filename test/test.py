import math
import time
from robot_systems.robot import HamBot

# Initialize the robot with default settings (LIDAR and Camera disabled for this task)



WHEEL_DIAMETER = 90  # mm
WHEEL_PERIMETER = WHEEL_DIAMETER * math.pi
AXLE_TRACK = 184   # mm

def straight(bot, distance, top_speed, starting_speed):
    """
    Moves the robot straight.
    """
    bot.reset_encoders()

    # Calculate the radians the wheels need to turn
    angle = (distance / WHEEL_PERIMETER) * 2 * math.pi
    speed = starting_speed
    print(f"Driving Straight: {distance}mm | Total radians: {angle:.1f}")
    
    bot.run_motors_for_seconds(1, starting_speed, starting_speed)
    # Drive loop
    while bot.get_left_encoder_reading() < angle:
        # Accelerating
        if (speed <= top_speed):
            speed += 5

        bot.set_left_motor_speed(speed)
        bot.set_right_motor_speed(speed)

        # Deceleratig
        if ((bot.get_left_encoder_reading() >= (angle * 0.8)) and (speed >= starting_speed)):
            speed -= 5
        
        
    
    bot.stop_motors()

def rotateInPlace(bot, degree, direction, top_speed, starting_speed):
    """
    Rotates the robot in place.
    Args:
        degree: The angle for the ROBOT to turn.
        direction: 1 = Right, -1 = Left.
        speed: Positive motor speed.
    """
    bot.reset_encoders()

    angle = degree / 360 * AXLE_TRACK * math.pi / WHEEL_PERIMETER * 2 * math.pi
    speed = starting_speed
    print(f"Rotating {'Right' if direction == 1 else 'Left'} {degree}ÃÂ° | Wheel Rotations: {angle:.1f}")

    # 3. Determine motor speeds based on direction
    # Direction 1 (Right): Left motor forward (-), Right motor backward (-)
    # Direction 0 (Left): Left motor backward (+), Right motor forward (+)
    if direction == 1:
        bot.run_motors_for_seconds(1, starting_speed, -starting_speed)
        while bot.get_left_encoder_reading() < angle:
            # Accelerating
            if (speed <= top_speed):
                speed += 2
            
            bot.set_left_motor_speed(speed)
            bot.set_right_motor_speed(-speed)

            if ((bot.get_left_encoder_reading() >= (angle * 0.8)) and (speed >= starting_speed)):
                speed -= 2
    else:
        bot.run_motors_for_seconds(1, -starting_speed, starting_speed)
        while bot.get_right_encoder_reading() < angle:
            # Accelerating
            if (speed <= top_speed):
                speed += 2
            
            bot.set_left_motor_speed(-speed)
            bot.set_right_motor_speed(speed)
            print(bot.get_right_encoder_reading())

            if ((bot.get_right_encoder_reading() >= (angle * 0.8)) and (speed >= starting_speed)):
                speed -= 2

    bot.stop_motors()
    
def circle(bot, radius, direction, outer_speed):
    angular_velocity = WHEEL_DIAMETER / 2 * outer_speed / (radius + AXLE_TRACK / 2)
    inner_speed = angular_velocity * (radius -  AXLE_TRACK / 2) / WHEEL_DIAMETER * 2
    outer_rotation = (2 * radius + AXLE_TRACK) * math.pi / WHEEL_PERIMETER
    inner_rotation = (2 * radius - AXLE_TRACK) * math.pi / WHEEL_PERIMETER
    
    print({inner_speed}, {outer_speed}, {angular_velocity})
    print({inner_rotation}, {outer_rotation})
    
    if direction == 1:
        bot.run_left_motor_for_rotations(inner_rotation, inner_speed, False)
        bot.run_right_motor_for_rotations(outer_rotation, outer_speed, True)
    else:
        bot.run_right_motor_for_rotations(inner_rotation, inner_speed, False)
        bot.run_left_motor_for_rotations(outer_rotation, outer_speed, True)
        
def rectangle(length, width, speed):
    straight(my_robot, width / 2, speed)
    time.sleep(1)
        
    rotateInPlace(my_robot, 90, 1, speed)
    time.sleep(1)
        
    straight(my_robot, length, speed)
    time.sleep(1)
    
    rotateInPlace(my_robot, 90, 1, speed)
    time.sleep(1)
    
    straight(my_robot, width, speed)
    time.sleep(1)
    
    rotateInPlace(my_robot, 90, 1, speed)
    time.sleep(1)
    
    straight(my_robot, length, speed)
    time.sleep(1)
    
    rotateInPlace(my_robot, 90, 1, speed)
    time.sleep(1)
    
    straight(my_robot, width / 2, speed)
    time.sleep(1)


# --- Test Routine ---
if __name__ == "__main__":
    try:
        my_robot = HamBot(lidar_enabled=False, camera_enabled=False)
        
        straight(my_robot, 630, 50,  0)
        time.sleep(0.5)

        rotateInPlace(my_robot, 180, 0, 50, 0)
        time.sleep(0.5)



    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        if 'my_robot' in locals():
            my_robot.disconnect_robot()
