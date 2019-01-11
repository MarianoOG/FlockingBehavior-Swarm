# Swam

This thesis work shows an application in autonomous aerial robots (UAV) that behave like a flock following a swarm particle optimization algorithm (PSO) adaptation and Reynolds First Model for trajectory generation. The task to solve is a simultaneous localization and mapping (SLAM) technique for monocular vision using a digital camera and an inertial measurement unit (IMU) is used. This project is made on ROS and Gazebo using python to program the nodes of the software.

The drones have the following characteristics that make them act like a swarm: Autonomous, able to measure with sensors and act in the environment,  control rules must allow a large number of robots, the robots must be homogeneous or very similar to each other. Each robot is inefficient to perform the mapping and we try to improve its performance when acting in a group.

In this project we: Design a movement control for each unit, implement recognition and communication protocols between robots, implement a simultaneous localization and mapping algorithm (SLAM) for monocular vision, implement the swarm particle algorithm, along other tasks.

The drones follow three main directives to act as a flock: short-range repulsion, to avoid collisions; alignment: to avoid oscillations; global position restriction, to keep individual robots together and act as a whole. 

The general steps followed by the drones to the construction of the map are: the drones will start from a known position and orientation, after the μPSO algorithm is started, each of the drones will create an independent map until the nominal convergence condition is reached, the drones will be returned to the initial position to conclude the closing cycle of said maps, the process for the unification of the maps will begin, restart the herd in random positions that offer new information and that are known within the map, we continue with the algorithm μPSO updating the complete map to each internal cycle. It ends when the general detention condition is reached.

To carry out the simulations in Gazebo, the routines of the hector_quadrotor package of the Technical University of Darmstadt created at the end of 2008 were be adapted. At the moment, ORB-SLAM seems to be the best option to implement monocular SLAM, since it is based on the characteristics, it presents the invariance to the point of view and the changes in the illumination.

This project is still in progress since the physical drones are still not adapted completely and some of the parts in the algorithm must be adapted to fully interact with each other. However, this is a proof of concept on flocking behavior for mapping.
