# Requirements
- Core Objective: Implement the Bug 0 algorithm to navigate the robot autonomously to a red cylinder goal marker while avoiding obstacles.
- Environment Setup:
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab3_physical_map.png">

- Sensor Utilization: Use the onboard Raspberry Pi camera to detect the goal landmark via RGB color matching, and use LIDAR to detect obstacles.
- State Machine Logic: The controller must alternate between two states:
  + Motion to Goal: Drive directly toward the yellow cylinder until an obstacle blocks the path.
  + Wall Following: Follow the obstacle boundary until the line-of-sight to the goal is clear again.
- Success Criteria: The robot must reach the goal from any starting location and stop within 0.25 m without colliding with walls or obstacles.

# Demonstration

<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab3_demo.gif">

# Take-away
- It's important to print out the values the robot is processing to debug. For instance, my robot constantly misses the red cylinder goal marker because the RGB values from the camera are different than the RGB values I provided.
