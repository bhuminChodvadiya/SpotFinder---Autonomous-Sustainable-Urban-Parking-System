# Component Description

`Suraj Yadav`

## Overview

Retrieve traffic data, including density and flow, through the Vehicle-to-Everything (V2X) Interface. Process the acquired data and generate outputs for analysis. This involves gathering real-time information on traffic conditions, analysing vehicle density, and assessing traffic flow to provide valuable insights for transportation management and planning.

## Component Architecture

![Block Diagramm](resource/image/tf.jpg)

## ROS2 Topics

| IN/Out | Topic Name  | Message Type | Description |
|--------|-------------|--------------|-------------|
| Input  | /all_vehicles_position  | sf_msgs/msg/ VehiclesPosition   | Retrieving traffic density and flow            |
| Output | /traffic_density | sf_msgs/msg/StreetInfoContainer    | Calculating the number of vehicles in between two nodes  |


## Functionality

The `sf_traffic_monitor` component subscribes to the `/all_vehicles_position` topic for real-time vehicle Positions and publishes to the `/traffic_density` topic. This node calculates the traffic density for each street segment in a map based on the positions of vehicles. The `calculate_density` method is called periodically with a period of 0.1 seconds and calculates the traffic density for each street segment by dividing the number of vehicles in the segment by the area of the segment. The `vehicles_position_callback` method is called whenever a new VehiclesPosition message is received on the `/all_vehicles_position` topic and updates the position of the vehicle in the `vehicles_positions` list. The node also initializes a list `vehicles_positions` to store the positions of vehicles and two dictionaries `name_dict` and `area_dict` to map street node names and distances between two nodes in the map street. Further this data is being subscribed by sf_path_plan for further functionality.

## Dependency

1. [sf_msgs](https://git.hs-coburg.de/SpotFinder/sf_msgs.git).

2. [sf_v2x_server](https://git.hs-coburg.de/SpotFinder/sf_v2x_server.git).

3. [sf_transmit](https://git.hs-coburg.de/SpotFinder/sf_transmit.git). 

## Installation
This repository needs to be cloned, built and sourced. It needs to be cloned to `/src` folder of the ROS2 workspace. This can be done by following the instruction provided in the [sf_master](https://git.hs-coburg.de/SpotFinder/sf_master.git).

## Run Steps

follow run steps in the [sf_master](https://git.hs-coburg.de/SpotFinder/sf_master.git)

1. Follow the below command to run the dependent component `cam_server`.

```bash
ros2 run sf_v2x_server cam_server 
```

2. Follow the below command to run the dependent component `receiver`.

```bash
ros2 run sf_v2x_com receiver 
```

3. Follow the below command to run `traffic_monitor` component. 

```bash
ros2 run sf_traffic_monitor traffic_monitor
```

# Model City Map

![model_city_map](resource/image/Model_Map.png)
