# ROS Project

This repository contains my ROS project for the Robot Programming class-DMAP Based Localization.

**This is a placeholder description**

DMAP localization refers to a type of localization technique in robotics that uses a distance map (DMAP) for determining the robot's position within a given environment. The distance map is a precomputed map where each cell contains the distance to the nearest obstacle.

### Key Koncepts:
1. Distance Map (DMAP):

	* A grid map where each cell value represents the distance to the nearest obstacle.
	* Useful for efficient collision checking and path planning.

2. Localization:
	* The process of determining the robot's position and orientation (pose) within the environment.
	* Often involves matching sensor data (e.g., laser scans, LiDAR) to the map. 

## Installation

1. Clone the repository:

	```bash
	git clone https://github.com/vmazashvili/ros_project.git
	cd ros_project

2. Build the workspace:
	```bash
	cd ~/catkin_ws
	catkin_make
	source devel/setup.bash
## Usage

To run the talker and listener nodes:

1. Open a terminal and run the talker node:

	```bash
	rosrun beginner_tutorials talker

2. Open another terminal and run the listener node:

	```bash
	rosrun beginner_tutorials listener

## Contributing

Feel free to fork this repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.

## License 

MIT 

