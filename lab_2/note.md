3 functions:
- Use LiDar to stop the robot 1 cell from the opposite wall (stop the loop when within certain distance from the wall)
- Use encoder to calculate how far it has gone and get slower as it reach the goal
- Rotating in place: use IMU for feedback control (Use relative degrees)

Make sure to have saturation
Tweak the PID combination (high Kp, significantly low Ki Kd)