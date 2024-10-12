### Integration Test Report for Mobile Interface (MI)

Date: [18 june,2024]

### Test Environment:

ROS Version: ROS 2 Foxy

### Introduction:
This report documents the integration testing results for the Mobile Interface (MI) which is designed to integrate with the Driving_controller and Localization components in a simulated vehicle environment.

### Objective:
The main objective of these tests is to verify that the Mobile Interface correctly handles and reacts to messages from the Localization component and interacts appropriately with the Driving_controller.

### Test Setup:

Mobile Interface Node
Driving Controller Node
Localization Node
Configured to simulate vehicle positions and movement.
Test Node
Publishes predefined messages to simulate EgoPosition and parking_reached_msg.


### Test Cases:

### Test Case ID: IT_MI_001

Description: Test EgoPosition update handling
Preconditions: All nodes initialized and running
Test Steps:
Publish EgoPosition with x=10.0, y=5.0, z=0.0
Verify if MI updates its internal state
Expected Results: MI should update its ego_position with the new values
Actual Results: [node is updating current position]
Status: [Pass]

### Test Case ID: IT_MI_002

Description: Test parking position reached message handling
Preconditions: All nodes initialized and running
Test Steps:
Publish Bool true to /parking_reached_msg
Verify if MI recognizes the parking position reached
Expected Results: MI should update its parking_reached_msg to true
Actual Results: [node is updating status]
Status: [Pass]

Total Tests: 2
Passed: [2]
Failed: [0]
Success Rate: [97]%

### Conclusions:
Based on the outcomes of the integration tests,it is integrating both of the components.

