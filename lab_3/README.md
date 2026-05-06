# Requirements
- Core Objective: Implement the Bug 0 algorithm to navigate the robot autonomously to a yellow cylinder goal marker while avoiding obstacles.
- Environment Setup: Construct a physical maze matching the layout shown in Figure 2 (II) of the documentation.
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/Robot%20-%20README.JPG">
- Sensor Utilization: Use the onboard Raspberry Pi camera to detect the goal landmark via RGB color matching, and use LIDAR to detect obstacles.
- State Machine Logic: The controller must alternate between two states:
  + Motion to Goal: Drive directly toward the yellow cylinder until an obstacle blocks the path.
  + Wall Following: Follow the obstacle boundary until the line-of-sight to the goal is clear again.
- Success Criteria: The robot must reach the goal from any starting location and stop within 0.25 m without colliding with walls or obstacles.

# Demonstration

# Take-away