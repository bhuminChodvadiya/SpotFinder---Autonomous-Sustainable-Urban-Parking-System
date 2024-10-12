# Traffic Monitor Integration test(IT_TM)

### Purpose
The integration test aims to verify the interaction and data flow between the TrafficMonitor node, its subscribers, and publishers. The test ensures that the TrafficMonitor correctly processes incoming vehicle position data and publishes accurate traffic density information.

### Topics:
`Subscriber`: /all_vehicles_position 

`Publisher`: /traffic_density 


## Test Case 1: Multiple Vehicle Positions Update(IT_TM_001)

### Precondition
- ROS2 system is initialized and the `TrafficMonitor` node is running.
- The `vehicles_positions` list is empty initially.

### Test Steps
1. Publish a `VehiclesPosition` message with `vehicle_id=1`, `x_coordinate=0.5`, and `y_coordinate=1.5` to the `/all_vehicles_position` topic.
2. Publish another `VehiclesPosition` message with `vehicle_id=2`, `x_coordinate=3.5`, and `y_coordinate=3.5` to the `/all_vehicles_position` topic.
3. Wait for the `calculate_density` method to be triggered by the timer.
4. Capture the published `StreetInfoContainer` message from the `/traffic_density` topic.

<img src="resource/image/MVPU.png" alt="Image" width="1000"/>

For image click on [link](https://git.hs-coburg.de/SpotFinder/sf_traffic_monitor/src/branch/main/resource/image/MVPU.png) 

### Expected Result
- The `vehicles_positions` list should contain two entries: 
  - `{"id": 1, "x_coordinate": 0.5, "y_coordinate": 1.5}`
  - `{"id": 2, "x_coordinate": 3.5, "y_coordinate": 3.5}`
- The `StreetInfoContainer` message should reflect:
  - A vehicle count of 1 on the street "JI" with an appropriate density calculation.
  - A vehicle count of 1 on the street "DE" with an appropriate density calculation.


# Test Case 2: No Vehicle Position Update(IT_TM_002)

### Precondition
- ROS2 system is initialized and the `TrafficMonitor` node is running.
- The `vehicles_positions` list is empty initially.

### Test Steps
1. Do not publish any `VehiclesPosition` message.
2. Wait for the `calculate_density` method to be triggered by the timer.
3. Capture the published `StreetInfoContainer` message from the `/traffic_density` topic.

<img src="../image/NVPU.png" alt="Image" width="1000"/>

For image click on [link](https://git.hs-coburg.de/SpotFinder/sf_traffic_monitor/src/branch/main/resource/image/NVPU.png) 

### Expected Result
- The `vehicles_positions` list should remain empty.
- The `StreetInfoContainer` message should reflect a vehicle count of 0 on all streets with densities calculated accordingly.


