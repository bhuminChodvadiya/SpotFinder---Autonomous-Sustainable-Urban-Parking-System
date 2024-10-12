# Component Description
| Author | Responsible |
|--------|-------------|
| `Rakshit shetty`   | Pure Pursuit Controller  |
| `Sandip Vasoya`    | Parking Maneuver         |
| `Suraj Yadav`      | Driving Controller       |

## Overview

The driving controller component is responsible for driving the vehicle and also parking vehicle to the chosen parking spot. It can determine the appropriate velocity and when to accelerate or decelerate during driving. Additionally, it has the capability to perform an emergency stop in situations such as the sudden appearance of obstacles, accidents, or collisions with cars, pedestrians, or other obstacles. Once it is reached within the range of parking spot, then it will start parking the car.                After the car is parked it will send confirmation message.   

## Component Architecture
![Block Diagramm](resource/driving_controller.jpg)

## ROS2 Topics


| IN/Out | Topic Name    | Message Type |              Description                          |
|--------|---------------|--------------|---------------------------------------------------|
| Input  | /ego_position | sf_msgs/msg/EgoPosition  |  Real-time ego vehicle's Data and Pose. |
| Input  | /planned_path | sf_msgs/msg/PathData     |  Path plan contains node id and its X-Y coordinates   |
| Input  | /env_model    | sf_msgs/msg/EMM          |  Fused data from Camera OD and LIDAR |
| Input  | /pp_cmd_vel   | geometry_msgs/msg/Twist  |  Provide velocity and steering angle for car which used for car maneuvering towards the location of parking|
| Input  | /pm_cmd_vel   | geometry_msgs/msg/Twist  |  Provide velocity and steering angle for car which is used for parking |
| Output | /cmd_vel      | geometry_msgs/msg/Twist  |  Car maneuver datas|
| Output | /parking_reached_msg  | std_msgs/bool | Car is parked |


## Functionality

### Pure Pursuit
The `pure_pursuit` algorithm calculates the steering angle required for the vehicle to follow a predefined path or set of waypoints. The node subscribes to topics for receiving the vehicle's current pose, velocity feedback, and the desired path. It publishes the calculated linear and angular velocity commands to the `/pp_cmd_vel` topic. The controller also includes a velocity PID controller to regulate the vehicle's speed.

### Parking Maneuver

The parking method utilizes basic geometric and trigonometric calculations to determine the angle needed for model car alignment. It computes the angle to the parking spot and adjusts for perpendicular alignment using straightforward trigonometric functions like atan2.
#### Vehicle and Parking Spot Position

The `parking_maneuver` subscribes to the `/ego_position` topic to receive the vehicle's current position and orientation. In this coordinate system, the vehicle's current position is marked as (x0, y0), and its yaw angle indicates the direction the vehicle is facing. The desired parking spot is located at (Xp, Yp), as shown in Fig. 1, which is based on the user-selected parking spot on the UI. To align the vehicle correctly with the parking spot, we first need to calculate the angle from the vehicle to the target spot. This angle, referred to as the `angle_to_target`, is computed using the atan2 function:

    angle_to_target = atan2(y_target - y_car, x_target - x_car)

This angle determines the direction towards the parking spot.

#### Calculating the Perpendicular Alignment

To park the vehicle correctly, it needs to be aligned perpendicularly to the parking spot. This perpendicular alignment ensures that the vehicle can move directly into the parking spot. We achieve this by adding 90 degrees to the `angle_to_target`:

    perpendicular_angle = angle_to_target + π/2

This angle represents the direction in which the vehicle should be aligned to be perpendicular to the parking spot.

#### Determining the Steering Angle

The steering angle required to align the vehicle correctly is the difference between the `perpendicular_angle` and the vehicle’s current yaw angle:

    steering_angle = perpendicular_angle - yaw_angle

This angle is then normalized to ensure it falls within the range suitable for the steering system:

    steering_angle_deg = np.degrees(steering_angle)

Finally, this angle is converted to degrees and limited to a practical range for the steering system:

    normalized_steering_angle_deg = max(-30.0, min(30.0, steering_angle_deg))


**Fig.1** 

<img src="../Experimental Setup.png" alt="Image" width="1000"/>


- if image are not see here , please follow this path : resource/Experimental Setup.png
#### Vehicle Movement and Alignment

As the vehicle moves, it first aligns itself with the parking spot's y-coordinate by moving forward, as shown in Fig. 2. Once the vehicle is aligned, it begins moving backward to enter the parking spot. The forward and backward movements are controlled by adjusting the linear velocity of the vehicle:

- Forward Movement: Adjusted by setting the linear velocity to 0.3 m/s while aligning with the y-coordinate.
- Backward Movement: Adjusted by setting the linear velocity to -0.3 m/s once aligned, guiding the vehicle into the parking spot.

It publishes the calculated twist commands (linear and angular velocities) to the `/pm_cmd_vel` topic, which can be used by a lower-level controller to execute the parking maneuver.

#### Distance Calculation

To finalize the parking maneuver, the vehicle needs to determine if it is close enough to the parking spot. The distance to the spot is calculated using:

    distance_to_spot = sqrt((x_car - x_target)^2 + (y_car - y_target)^2)

**Fig.2** 

<img src="../Figure.png" alt="Image" width="1000"/>

- if image are not see here , please follow this path : resource/Figure.png

When this distance is below a certain threshold, the vehicle adjusts its steering angle to ensure it is aligned correctly before stopping. After successfully parking, the script sends a boolean message to the Mobile Interface (MI) indicating that parking is complete.

### Driving Controller
The `sf_driving_controller` subscribes to various topics to receive data on the vehicle's position `/ego_position`, planned path `/planned_path`, final parking point `/final_parking_point`, and environment model `/env_model`. Additionally, it listens for control commands from the pure pursuit and parking maneuver controllers via `/pp_cmd_vel` and `/pm_cmd_vel`.

The class initializes several variables to store the vehicle's state, path data, control messages, and flags for emergency braking and parking status. The `car_movement` method, which is triggered by a timer, controls the vehicle's motion. This method checks for the availability of necessary data, handles emergency braking if required, and transitions the vehicle between different driving states, such as pure pursuit driving and parking maneuvers.

Emergency braking is determined by calculating the time-to-collision (TTC) based on the vehicle's current velocity and the distance to obstacles in the environment. If the TTC is less than or equal to 2 seconds, the vehicle decelerates to avoid a collision. The `decelerate` method sets the vehicle's velocity to zero.

The `reached_final_waypoint` method checks if the vehicle is close to the final parking point and triggers the parking maneuver if it is. The `parking_manoeuver` method executes parking commands.

Various callback methods (`position_callback`, `path_callback`, `final_parking_callback`, `environment_model_callback`, `parking_callback`, and `pure_pursuit_callback`) update the internal state of the vehicle based on incoming messages. The `publish_twist_message` method publishes the control commands to the `/cmd_vel` topic to actuate the vehicle's movements.


Refer [State diagram](https://git.hs-coburg.de/SpotFinder/sf_master/src/branch/main/resources/documentation/architecture.md#state-diagram) for vehicle state change.


## Requirements
1. Car's Remote Controller
2. [ros2_pcan](https://git.hs-coburg.de/Autonomous_Driving/ros2_pcan.git)
3. [V2X msg](https://git.hs-coburg.de/Autonomous_Driving/v2x.git)


## Dependencies

1. [sf_msgs](https://git.hs-coburg.de/SpotFinder/sf_msgs.git).
2. [sf_localization](https://git.hs-coburg.de/SpotFinder/sf_localization.git)
3. [sf_path_plan](https://git.hs-coburg.de/SpotFinder/sf_path_plan.git)
4. [sf_env](https://git.hs-coburg.de/SpotFinder/sf_env.git)
5. [pure_pursuit](https://git.hs-coburg.de/SpotFinder/sf_driving_controller/src/branch/main/sf_driving_controller/pure_pursuit.py)
6. [parking_maneuver](https://git.hs-coburg.de/SpotFinder/sf_driving_controller/src/branch/main/sf_driving_controller/parking_manoeuver.py#L21)

## Installation

This repository needs to be cloned, built and sourced. It needs to be cloned to `/src` folder of the ROS2 workspace. This can be done by following the instruction provided in the [sf_master](https://git.hs-coburg.de/SpotFinder/sf_master.git).

## Run Steps

follow run steps in the [sf_master](https://git.hs-coburg.de/SpotFinder/sf_master.git)

1. Follow the below command to run the `ros2pcan_node`.
```bash
ros2 run ros2_pcan ros2pcan_node
```

2. Follow the below command to run the `sf_ego_localization`.
```bash
ros2 run sf_localization sf_ego_localization
```

3. Follow the below command to run the `path_plan`.
```bash
ros2 run sf_path_plan path_planning
```

4. Follow the below command to run the `env_model`.
```bash
ros2 run sf_env env_model
```

5. Follow the below command to run the `pure_pursuit`.
```bash
ros2 run sf_driving_controller pure_pursuit
```

6. Follow the below command to run the `parking`.
```bash
ros2 run sf_driving_controller parking
```

7. Follow the below command to run the `driving_controller` component.
```bash
ros2 run sf_driving_controller driving_controller
```
## Video: Pure Pursuit 
- Your browser does not support the video tag. go into [here](src/sf_driving_controller/resource/integration_test_parking_manuver/test_parking_maneuver.mp4)

## Video: Parking Maneuver
- Your browser does not support the video tag. go into [here](src/sf_driving_controller/resource/integration_test_parking_manuver/test_parking_maneuver.mp4)



