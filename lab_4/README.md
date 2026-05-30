# Requirements
## Task 1: Trilateration
- Core Objective: Localize the robot by calculating continuous $(x,y)$ coordinates and mapping them to a grid cell index (1-25) using trilateration.  
- Environment Setup: Construct Physical Maze 1 with four colored landmarks (Yellow, Red, Green, Blue) placed at the grid corners.
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab4_task1_physical_map.png">

- Sensor Utilization: Use the onboard camera to detect colored landmarks and the LIDAR to measure distances to at least three of these landmarks.
- Logic and Execution: Start in a random initial cell, measure distances, apply trilateration equations to estimate position, and print the coordinates and cell index to the console.
- Success Criteria: Accurately calculate the robot's $(x,y)$ coordinates and correct cell index across all starting locations, completing the computation once per starting location.

<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab4_task1_demo.gif">

## Task 2: Particle Filter
- Core Objective: Implement grid-based localization using a particle filter to estimate the robot's cell location based on wall observations.
- Environment Setup: Construct Physical Maze 2 using a $5\times5$ grid where each cell has a specific wall signature $(N, E, S, W)$ representing the presence of boundaries. 
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab4_task2_physical_map.png">

- Sensor Utilization: Use LIDAR to observe surrounding walls and the IMU to determine the robot's heading relative to its starting orientation.
- Logic and Execution:
  + Initialize 250 particles distributed evenly across the grid.
  + Execute the filter cycle: predict state using a perfect deterministic motion model, correct particle weights using a provided noisy sensor model, normalize the weights, and resample particles with randomized orientations.
  + Print the particle distribution, the mode cell estimate, and the convergence status after each update step.
- Success Criteria: Accurately predict the robot's location across random starts and terminate the algorithm once 80% or more of the particles converge into a single cell.

<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab4_task2_demo.gif">
