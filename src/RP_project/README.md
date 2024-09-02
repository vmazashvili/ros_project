### README

## Project Overview

This ROS node is designed to perform robot localization using laser scan data in a predefined map. The node subscribes to the following topics:
- `/map`: Provides the occupancy grid representing the environment.
- `/initialpose`: Provides the initial pose of the robot within the map.
- `/base_scan`: Receives laser scan data from the robot's LiDAR sensor.

The node processes this data to localize the robot within the map by matching laser scan data with the map data and updating the robot's position.

## Prerequisites

Ensure that you have the following setup:
- A ROS workspace (`catkin_ws`).
- Necessary ROS packages and dependencies.
- The custom libraries and headers required for the project, which should be located in the `/rp_files` folder provided in the Robot Programming class.

## Setup Instructions

1. **Move the Project Files**: Ensure that all the project files, including the custom headers and libraries (`dmap.h`, `grid_map.h`, `dmap_localizer.h`, etc.), are placed in the `/rp_files` folder inside your `src` directory of the `catkin_ws` workspace.

2. **Compile the Project**:
   - Open a terminal and navigate to your `catkin_ws` directory.
   - Compile the workspace using:
     ```bash
     catkin_make
     ```

3. **Launch the ROS Master**:
   - In a separate terminal, start the ROS master node by running:
     ```bash
     roscore
     ```

4. **Run the Localization Node**:
   - In another terminal, navigate to your `catkin_ws` directory and run the node using:
     ```bash
     rosrun project_localizer main_node
     ```

   - After launching, the node should receive the map, create a DMap, and start listening for the initial pose and laser scan data.

5. **Check ROS Topics**:
   - To verify that the node is correctly subscribing to the necessary topics, you can list all active topics using:
     ```bash
     rostopic list
     ```
   - To inspect the messages received by a particular topic, use:
     ```bash
     rostopic echo <topic_name>
     ```
     For example, to see the data from the `/base_scan` topic, use:
     ```bash
     rostopic echo /base_scan
     ```

## Notes

- **Visualization**: You can use RViz to visualize the map, laser scan data, and the robot's pose for better debugging and analysis.
- **Troubleshooting**: Ensure that your topics are correctly named and the data being published matches the expected format. Misalignment in map resolution or laser scan parameters can affect localization accuracy.
- **Custom Libraries**: The custom libraries necessary for this project are located in the `/rp_files` folder provided in the Robot Programming class. Ensure these are correctly included and linked in your `CMakeLists.txt`.

With these steps, your ROS localization node should function as expected, helping you achieve accurate robot localization within a predefined map using laser scan data.