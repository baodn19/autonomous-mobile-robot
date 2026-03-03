"""
Author: Ngoc Bao Dinh
UID: U27463715

AI
Prompt 1:
Program a Python script to fulfill the tasks outline for physical robots in the pdf "Lab 2 Code Submission.pdf".

For physical robots information, refer to "RobotHardware2026.pdf".

For the functions that can be implement in the Python script, refer to "robot.py"

For LiDAR functions, refer to "lidar.py"

Prompt 2:
Why does the robot only turn clockwise no matter the sign of the angle?

Usage: It gave me a structure that I followed. I still had to re-tune the PID coefficients to get the robot to turn correctly. I also had to add a special case for exact 180-degree turns to preserve the direction of rotation. The original code was treating 180-degree errors as zero, which caused it to always choose the same rotation direction. By using math.copysign, I ensured that the error for 180-degree turns has the correct sign based on the desired rotation direction.
"""

import time
import math
from robot_systems.robot import HamBot

DT = 0.032 # fixed timestep of 32ms
WHEEL_RADIUS_MM = 45.0  
SATURATED_RPM = 75.0 
LIDAR_FRONT_INDEX = 180 
FT_TO_MM = 304.8

"""
    Function: saturate the control signal to max RPM and apply deadband
    Parameter:
    - control_signal: pre-saturated control
    - min_rpm: minimum RPM threshold for deadband
"""
def saturate(control_signal, min_rpm=20.0):
    if abs(control_signal) < min_rpm:
        if control_signal < 0:
            return -min_rpm
        else:
            return min_rpm
        
    return max(-SATURATED_RPM, min(SATURATED_RPM, control_signal))

"""
    Function: travel straight using LIDAR feedback to maintain distance from wall
    Parameter:
    - robot: HamBot instance
    - target_ft: desired distance from wall in feet
    - kp, ki, kd: PID coefficients
    - max_accel_rpm_s: maximum acceleration in RPM/s for acceleration limiting
"""
def straight_lidar(robot, target_ft, kp, ki, kd, max_accel_rpm_s=150.0):
    target_mm = target_ft * FT_TO_MM
    integral = 0.0
    prev_error = 0.0
    current_rpm = 0.0 

    while True:
        start_time = time.time()
        scan = robot.get_range_image()
        
        if scan != -1 and scan[LIDAR_FRONT_INDEX] > 0:
            current_distance_mm = scan[LIDAR_FRONT_INDEX]
            
            error = current_distance_mm - target_mm
            
            # I(t) = I(t−1) + e(t)·dt
            integral += error * DT
            # D(t) = (e(t) − e(t−1)) / dt
            derivative = (error - prev_error) / DT
            
            control = (kp * error) + (ki * integral) + (kd * derivative)
            prev_error = error
            
            target_rpm = saturate(control)
            
            # Acceleration limiting
            max_rpm_change = max_accel_rpm_s * DT
            if target_rpm > current_rpm + max_rpm_change:
                current_rpm += max_rpm_change
            elif target_rpm < current_rpm - max_rpm_change:
                current_rpm -= max_rpm_change
            else:
                current_rpm = target_rpm
            
            print(current_rpm, " ", error)
            robot.set_left_motor_speed(current_rpm)
            robot.set_right_motor_speed(current_rpm)
            
            # Stop condition: within 20mm and low velocity
            if abs(error) < 20.0 and abs(derivative) < 20.0:
                robot.stop_motors()
                break
                
        time.sleep(max(0.0, DT - (time.time() - start_time)))

"""
    Function: travel straight using encoder feedback to monitor distance traveled
    Parameter:
    - robot: HamBot instance
    - target_ft: desired distance from wall in feet
    - kp, ki, kd: PID coefficients
"""
def straight_encoder(robot, target_ft, kp, ki, kd):
    target_mm = target_ft * FT_TO_MM
    robot.reset_encoders()
    integral = 0.0
    prev_error = target_mm

    while True:
        start_time = time.time()
        
        left_rad, right_rad = robot.get_encoder_readings()
        avg_radians = (left_rad + right_rad) / 2.0
        distance_traveled_mm = avg_radians * WHEEL_RADIUS_MM
        
        error = target_mm - distance_traveled_mm
        
        integral += error * DT
        derivative = (error - prev_error) / DT
        
        control = (kp * error) + (ki * integral) + (kd * derivative)
        prev_error = error
        
        rpm = saturate(control)
        print(rpm, " ", error)
        robot.set_left_motor_speed(rpm)
        robot.set_right_motor_speed(rpm)
        
        # Stop condition: within 20mm and low velocity
        if abs(error) < 20.0 and abs(derivative) < 20.0:
            robot.stop_motors()
            break
            
        time.sleep(max(0.0, DT - (time.time() - start_time)))

"""
    Function: rotate in place using IMU feedback to monitor heading
    Parameter:
    - robot: HamBot instance
    - target_angle_deg: desired angle in degrees (negative for clockwise)
    - kp, ki, kd: PID coefficients
"""
def rotate_imu(robot, target_angle_deg, kp, ki, kd):
    start_heading = robot.get_heading(blocking=True)
    if start_heading is None:
        print("Failed to get IMU heading.")
        return

    target_heading = (start_heading + target_angle_deg) % 360.0
    integral = 0.0
    prev_error = 0.0

    while True:
        start_time = time.time()
        current_heading = robot.get_heading(blocking=True)
        
        if current_heading is not None:
            error = (target_heading - current_heading + 180.0) % 360.0 - 180.0
            
            # Preserve direction for exact 180-degree turns
            if abs(error) == 180.0:
                error = math.copysign(180.0, target_angle_deg)
            
            integral += error * DT
            derivative = (error - prev_error) / DT
            
            control = (kp * error) + (ki * integral) + (kd * derivative)
            prev_error = error
            
            rpm = saturate(control)
            print(rpm, " ", error)
            robot.set_left_motor_speed(-rpm)
            robot.set_right_motor_speed(rpm)
            
            # Stop condition: within 3 degree and low angular velocity
            if abs(error) < 3.0 and abs(derivative) < 20.0:
                robot.stop_motors()
                break
                
        time.sleep(max(0.0, DT - (time.time() - start_time)))

def main():
    robot = HamBot()
    time.sleep(2)  # Wait for sensors to initialize
    
    kp_lidar, ki_lidar, kd_lidar = 0.075, 0.0, 0.01
    kp_enc, ki_enc, kd_enc = 0.2, 0.0, 0.01
    kp_imu, ki_imu, kd_imu = 0.5, 0.0, 0.01

    try:
        # Task 1: LIDAR PID - Stop 2 ft from wall
        straight_lidar(robot, 2.0, kp_lidar, ki_lidar, kd_lidar)
        time.sleep(1)

        # Task 2: Encoder PID - Drive 1 ft
        straight_encoder(robot, 1.0, kp_enc, ki_enc, kd_enc)
        time.sleep(1)

        # Task 3: LIDAR PID - Backup to 2 ft from wall
        straight_lidar(robot, 2.0, kp_lidar, ki_lidar, kd_lidar)
        time.sleep(1)

        # Task 4: IMU PID - Rotate -180 deg (Clockwise)
        rotate_imu(robot, -180.0, kp_imu, ki_imu, kd_imu)
        time.sleep(1)

        # Task 5: LIDAR PID - Stop 2 ft from wall
        straight_lidar(robot, 2.0, kp_lidar, ki_lidar, kd_lidar)
        time.sleep(1)

        # Task 6: Encoder PID - Drive 1 ft
        straight_encoder(robot, 1.0, kp_enc, ki_enc, kd_enc)
        time.sleep(1)

        # Task 7: LIDAR PID - Backup to 2 ft from wall
        straight_lidar(robot, 2.0, kp_lidar, ki_lidar, kd_lidar)
        time.sleep(1)

        # Task 8: IMU PID - Rotate +180 deg (Counterclockwise)
        rotate_imu(robot, 180.0, kp_imu, ki_imu, kd_imu)
        time.sleep(1)

        # Task 9: Encoder PID - Drive 2 ft
        straight_encoder(robot, 2.0, kp_enc, ki_enc, kd_enc)
        time.sleep(1)

    finally:
        robot.disconnect_robot()

if __name__ == "__main__":
    main()