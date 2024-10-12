# SpotFinder
## Overview
<div style="text-align: justify">
SpotFinder is an innovative and sustianable parking solution that eliminates the hassle of finding a parking spot. Designed to provide a seamless experience for users, SpotFinder ensures that once the destination is selected, the system autonomously locates a sustainable parking spot nearby. The entire process, from driving to parking, is automated, allowing users to relax and enjoy a stress-free parking experience.
</div>

 <br />
<div style="text-align:center">
    <img src="images/system_archi.jpg" alt="main_block_diagram" width="1000">
</div>


[Demo Video](https://git.hs-coburg.de/SpotFinder/sf_master/src/branch/main/resources/images/Demo.mp4)

## Table of Content

1. [Architecture Documentation](#1-architecture-document)
2. [Project Management](#2-project-management)
3. [Functionality](#1-functionality)
4. [Depenency](#2-depenency)
5. [Installation](#3-installation)
6. [Steps to Start SpotFinder](#4-steps-to-start-spotfinder)
7. [Steps to run component individually](#5-steps-to-run-component-individually)


## 1. [Architecture Document](resources/documentation/architecture.md)

## 2. [Project Management](resources/documentation/project_management.md)

## 3. Functionality
<div style="text-align: justify">
Spotfinder is designed to calculate the distance between available parking spots and the drop-off location, select the best parking spot around the drop-off location, drive the vehicle to the selected spot and park the vehicle. Finally, it sends a notification to the vehicle owner to let them know that their vehicle is parked safely at the selected location.
</div>

### [sf_env](https://git.hs-coburg.de/SpotFinder/sf_env.git)
<div style="text-align: justify">
This `sf_env` is responsible for creating a comprehensive and dynamic virtual model of the environment surrounding the vehicle based on incoming data from the camera, LIDAR and the current position. This component uses sensor data fusion in order to create an accurate Model of the environment.
</div>

### [sf_localization](https://git.hs-coburg.de/SpotFinder/sf_localization.git)
<div style="text-align: justify">
The `sf_localization` Component provides real-time vehicle data. It delivers X and Y coordinates in ModelCity format, pose/orientation of vehicle and geographic co-ordinates (latitude, longitude).
</div>

### [sf_driving_controller](https://git.hs-coburg.de/SpotFinder/sf_driving_controller.git)
<div style="text-align: justify">
The driving controller component is responsible for driving the vehicle and also parking vehicle to the chosen parking spot. It can determine the appropriate velocity and when to accelerate or decelerate during driving. Additionally, it has the capability to perform an emergency stop in situations such as the sudden appearance of obstacles, accidents, or collisions with cars, pedestrians, or other obstacles. Once it is reached within the range of parking spot, then it will start parking the car. After the car is parked it will send confirmation message.
</div>

### [sf_path_plan](https://git.hs-coburg.de/SpotFinder/sf_path_plan.git)
<div style="text-align: justify">
The `sf_route_plan` component is designed to plan a path. The PathPlan node subscribes to the `/ego_position` and `/parking_spots` topics to receive the vehicle's current position and available parking spots through EgoPosition and ParkingSpotContainer messages, respectively. It publishes the selected parking point and planned path on the `/final_parking_point` and `/planned_path` topics using SelectedParking and Path messages. The node periodically checks for parking spot availability, publishes the path, and the selected parking point using timers. It sets up clients for select_parking_spot and book_parking_spot services to select and book the nearest parking spot, employing the A* algorithm for pathfinding by parsing map data to determine the optimal path and calculating distances. The select_parking_spot method sorts and books the closest spot, publishing the path and parking point upon successful booking. Asynchronous service calls and efficient execution are managed using callback groups and a multi-threaded executor. The algorithm's Node class represents graph nodes with coordinates, costs, and parent relationships, with functions for parsing map data, calculating distances, finding the closest node, and computing heuristics. The astar function performs the A* algorithm, exploring nodes iteratively using a priority queue to find and reconstruct the shortest path to the end node.
</div>

### [sf_v2x_com](https://git.hs-coburg.de/SpotFinder/sf_v2x_com.git)
<div style="text-align: justify">
This component is responsible for retrieving the ego vehicle position and heading (yaw) and transmitting this data via V2X. It is also responsible for receiving V2X messages that are transmitted from other vehicles. This is achieved using the CAM messages. This component also transmits the detected objects from the re-trained detectnet via V2X as CPM messages.
</div>

### [sf_v2x_server](https://git.hs-coburg.de/SpotFinder/sf_v2x_server.git)
<div style="text-align: justify">
The `sf_v2x_server` is the central hub for V2X communication in the SpotFinder ecosystem. Its primary function is to provide real-time information to SpotFinder about available parking spots and traffic conditions.
</div>

### [sf_msgs](https://git.hs-coburg.de/SpotFinder/sf_msgs.git)
<div style="text-align: justify">
The `sf_msgs` contain custom messages to provide parking spot details, real-time position updates, traffic flow data, and more.
</div>

### [sf_model_city_map](https://git.hs-coburg.de/SpotFinder/sf_model_city_map.git)
<div style="text-align: justify">
The `sf_model_city_map` contain custom messages to provide parking spot details, real-time position updates, traffic flow data, and more.
</div>

### [sf_mi](https://git.hs-coburg.de/SpotFinder/sf_mi.git)
<div style="text-align: justify">
The `sf_mi` component is a mobile interface of SpotFinder that improves the user experience by simplifying the parking process and a parking confirmation notification. Users can stay informed about their vehicle status in real time and will receive a confirmation message once the parking process has been successfully completed.
</div>

### [sf_ui](https://git.hs-coburg.de/SpotFinder/sf_ui.git)
<div style="text-align: justify">
The `sf_ui` component serves as a vehicle display interface for user interaction with SpotFinder. It allows you to activate SpotFinder, choose a parking spot, and enjoy a seamless parking experience.
</div>


## 2. Depenency

Sofware Depenency:

1. [Ubuntu Version : Ubuntu 20.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
2. [Ros version: Ros2Foxy](https://docs.ros.org/en/foxy/Installation.html)
3. [Mocap msgs](https://github.com/ros-drivers/mocap_optitrack)
4. [V2X msg](https://git.hs-coburg.de/Autonomous_Driving/v2x.git)
5. [ROS2_PCAN](https://git.hs-coburg.de/Autonomous_Driving/ros2_pcan.git)
6. [Ros Deeplearning](https://git.hs-coburg.de/Autonomous_Driving/ros_deep_learning)
7. [Ydlidar Driver](https://git.hs-coburg.de/Autonomous_Driving/ydlidar_ros2)
8. [Realsense_Camera](https://github.com/IntelRealSense/realsense-ros)
 
Hardware Depenency:

1. Optitrack Mocap system
2. Intel-Realsense Camera
3. ydlidar G4

## 3. Installation


1. Clone Repository:
```bash
git clone https://git.hs-coburg.de/SpotFinder/sf_master.git
```
2. Clone Sub-Repository:
```bash
cd sf_master
vcs import src < sf_repos.repos
```
For further Dependency refer to the READme.md file of the Sub-repository.
## 4. Steps to Start SpotFinder
 
1. Start Launch-File
```bash
cd sf_master
ros2 launch launch.py
```
2. Starting detectnet
```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
ros2 launch ros_deep_learning detectnet.ros2.launch
```
3. Start YdLidar
```bash
ros2 run ydlidar ydlidar
```
4. Start PCAN
```bash
ros2 run ros2_pcan ros2_pcan
```
## 5. Steps to run component individually

1. Source `setup.bash` in ros2 work space
```bash
source ./install/setup.bash
```

2. Command for running component
```bash
ros2 run <Pkg-Name> <Executable-Name>
```
