### README

## Project Overview

This ROS node is designed to perform robot localization using laser scan data in a predefined map. The node subscribes to the following topics:

- `/map`: Provides the occupancy grid representing the environment.
- `/initialpose`: Provides the initial pose of the robot within the map.
- `/base_scan`: Receives laser scan data from the robot's LiDAR sensor.

The node processes this data to localize the robot within the map by matching laser scan data with the map data and updating the robot's position in real-time.

## Prerequisites

Ensure that you have the following setup:
- A ROS workspace (`catkin_ws`).
- The necessary ROS packages and dependencies.
- The custom libraries and headers required for the project, which should be located in the `/rp_files` folder provided in the Robot Programming class.
- A map file (e.g., `map.yaml` with its corresponding `.pgm`) for the map server.

## Setup Instructions

### 1. **Move the Project Files**:
Ensure that all the project files, including the custom headers and libraries (`dmap.h`, `grid_map.h`, `dmap_localizer.h`, etc.), are placed in the `/rp_files` folder inside your `src` directory of the `catkin_ws` workspace.

### 2. **Compile the Project**:
- Open a terminal and navigate to your `catkin_ws` directory:
  ```bash
  cd ~/catkin_ws
  ```
- Compile the workspace using:
  ```bash
  catkin_make
  ```

### 3. **Launching the System**:
This project requires multiple terminals to run different processes. Follow these steps to start the system:

#### Terminal 1: **Start ROS Master**
- Open the first terminal and start the ROS master node by running:
  ```bash
  roscore
  ```
- Leave this terminal running in the background.

#### Terminal 2: **Launch the Map Server**
- Open the second terminal and navigate to your `catkin_ws` directory:
  ```bash
  cd ~/catkin_ws
  ```
- Launch the map server with your predefined map (replace `map.yaml` with your actual map file path):
  ```bash
  rosrun map_server map_server path/to/map.yaml
  ```
- This will publish the map to the `/map` topic for the localization node to use.

#### Terminal 3: **Run the Localization Node**
- Open the third terminal and navigate to your `catkin_ws` directory:
  ```bash
  cd ~/catkin_ws
  ```
- Launch the localization node:
  ```bash
  rosrun project_localizer main_node
  ```

At this point, the localization node will start subscribing to the necessary topics and wait for laser scan data to perform localization.

### 4. **Check ROS Topics**:
You can use the following commands to verify that the node is correctly subscribed to the necessary topics:

- List all active topics:
  ```bash
  rostopic list
  ```
  
- To inspect the messages received by a particular topic, use:
  ```bash
  rostopic echo <topic_name>
  ```
  For example, to inspect the data from the `/base_scan` topic:
  ```bash
  rostopic echo /base_scan
  ```

### 5. **Waiting for Laser Scan Data**:
Since this project relies on real-time laser scan data from the robot's LiDAR sensor, the node will wait for laser scans to be published on the `/base_scan` topic to perform localization. Without incoming laser scan data, the localization process will not begin. If you do not have a simulation or real hardware running, the system will appear idle as it awaits laser scan input.

### 6. **Visualization**:
You can use RViz to visualize the map, laser scan data, and the robot's pose for better debugging and analysis. To launch RViz:

- Open a new terminal and run:
  ```bash
  rosrun rviz rviz
  ```

In RViz:
- Add the `/map` topic to display the occupancy grid.
- Add the `/base_scan` topic to visualize the laser scans.
- Add the `/initialpose` and `/odom` topics (if available) to track the robot's pose and movement.

### 7. **Troubleshooting**:

- **Check Topic Names**: Ensure the topic names for `/map`, `/initialpose`, and `/base_scan` match the expected names in the system. You can adjust the topic names in the node if they differ.
- **Map Alignment**: Ensure that the resolution and size of the map match the real-world dimensions and that the laser scan range is configured appropriately.
- **Missing Laser Scans**: If the node does not localize, check if laser scan data is being published. Without laser scan input, the localization algorithm will not function.

### 8. **Custom Libraries**:
The project relies on custom libraries located in the `/rp_files` folder provided during the Robot Programming class. Ensure that these libraries (`dmap.h`, `grid_map.h`, `dmap_localizer.h`, etc.) are properly included in the project and linked in the `CMakeLists.txt` file.

### 9. **Additional Commands**:

- **Relocalizing the Robot**:
  If needed, you can relocalize the robot by publishing a new initial pose via the `/initialpose` topic. This can be done either programmatically or via RViz by manually setting the robot’s initial pose.

## Example Workflow Summary:

1. **Terminal 1**: Start `roscore`.
2. **Terminal 2**: Launch the map server with the command:
   ```bash
   rosrun map_server map_server path/to/map.yaml
   ```
3. **Terminal 3**: Run the localization node:
   ```bash
   rosrun project_localizer main_node
   ```
4. **Wait for Laser Scans**: The node will wait for incoming laser scan data to localize the robot.
5. **(Optional)**: Launch RViz for visualization and debugging:
   ```bash
   rosrun rviz rviz
   ```

## Conclusion:
With this setup, your ROS localization node should function correctly, localizing the robot within a predefined map using real-time laser scan data. The project is designed for a real-world setup where the robot uses its LiDAR sensor to continuously update its position within the environment.
