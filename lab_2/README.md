# Requirements
- Core Objective: Control motion using LIDAR for wall distance, wheel encoders for linear distance traveled, and an IMU/compass for rotation.
- Environment Setup:
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab2_physical_map.png">

- Desired Path:
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab2_desired_paths.png">

- Sensor Utilization: Execute the following nine-step sequence in exact order:
1. Use LIDAR PID to drive forward and stop 1 m from the wall.
2. Use encoder PID to drive forward 0.5 m and stop.
3. Use LIDAR PID to back up without rotating and stop 1 m from the wall.
4. Use IMU PID to rotate $-180^\circ$ clockwise and stop.  
5. Use LIDAR PID to drive forward and stop 1 m from the wall.
6. Use encoder PID to drive forward 0.5 m and stop.
7. Use LIDAR PID to back up without rotating and stop 1 m from the wall.
8. Use IMU PID to rotate $+180^\circ$ counterclockwise and stop.
9. Use encoder PID to drive forward 2.5 m and stop.



# Demonstration

# Take-away
- Finding the right combination of PID is a long process of trials and errors
- LIDAR PID is the most accurate among the LIDAR, IMU, and PID
- I have to account for reseting the angle whenever the robot does a full rotation in a place.
