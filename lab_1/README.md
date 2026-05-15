# Requirements
- Core Objective: Program a physical robot to navigate three continuous paths within an empty environment: a rectangle, a counterclockwise circle, and a clockwise circle.
- Desired Path:
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab1_desired_paths.png">

- Required Parameters (meters):

| L     | W   | $R_1$             |  $R_2$ |  
| ----- | --- | --------------- | --------------- |
| 1    | 2   | 0.5   | 1 |
| 1.5     | 0.5   | 0.75   | 0.25|


- Sensor Utilization: Utilize only encoders and compass sensors, manually defining the initial North heading.



# Demonstration
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab1_demo.gif">

# Take-away
- Small errors from friction and inertia can add up and significantly impact the robot's intended position
- External factors above can make the wheel encoder's measurement inaccurate
