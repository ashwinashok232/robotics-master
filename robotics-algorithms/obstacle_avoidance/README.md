# Motion Planning and Obstacle Detection

## Description
Given a simulated robot with a laser range finder built in, and a world file that contains various obstacles. Performs RANSAC on laser range finder data for 
obstacle detection (visualized using rviz on ROS). Then, uses the Bug2 algorithm in conjunction with obstacle detection for motion planning to reach goal coordinate.

## Outputs
Use peception.launch (in launch folder) to get obstacle detection results, and use bug2.launch to deploy algorithm on robot.
