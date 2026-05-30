# Requirements
- Core Objective: Develop a path planning algorithm to compute and execute the shortest valid path from a given start cell to a predefined goal cell.
- Environment Setup: Construct the approved physical maze, representing a grid world with internal walls.
<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab4_task2_physical_map.png">

- Algorithm/Logic:
  + Create a data structure to encode the grid and internal wall adjacency constraints.
  + Implement either a Wavefront Planner (Breadth-First Search) or Dijkstra's Algorithm.
  + Restrict movements to 4-connected directions (North, East, South, West) with a cost of 1 per move.
  + Print the calculated path to the console as an ordered list of cells and output the total path length in steps.
  + Traverse the path physically, moving cell-by-cell until reaching the goal.
- Success Criteria: The robot must autonomously compute the shortest path, display the correct console outputs, and physically reach the goal cell across multiple starting locations within a single, continuous, uninterrupted video recording.

# Demonstration

<img src="https://github.com/baodn19/autonomous-mobile-robot/blob/main/assets/lab5_demo.gif">
