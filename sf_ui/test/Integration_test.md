# Test Case 1: GUI Initialization and Layout Validation

### Precondition:
- ROS2 is properly set up.
- The required dependencies (`rclpy`, `PySimpleGUI`, `PIL`, `sf_msgs.srv.SelectSpot`) are installed.
- The image and font files (`mc_map.png`, `OpenSans-Medium.ttf`) exist at specified paths.

### Test Step:
1. Start the ROS2 node by running the script.
2. Simulate the `select_parking_spot` service call with a request containing a list of parking spots with predefined coordinates and distances.

### Expected Result:
- The GUI window titled "SpotFinder" should open.
- The window should display the modified map image (`mc_map_bbox.png`) with highlighted parking spots.
- The radio buttons should correctly list parking spots with accurate distances.
- The "confirm" button should be present.

# Test Case 2: Spot Selection and Response

### Precondition:
- ROS2 node is running with the `UserInterface` node active.
- The GUI is displayed with the correct layout.

### Test Step:
1. Simulate the `select_parking_spot` service call with a request containing multiple parking spots.
2. Select a parking spot using the radio buttons.
3. Click the "confirm" button.

### Expected Result:
- The selected spot ID should be correctly set in the response.
- The node log should show a message indicating the selected spot ID.
- The GUI window should close after the selection.

# Test Case 3: Timeout Handling

### Precondition:
- The User Interface node is running and displaying the parking spots.
- The timeout duration is set (e.g., 300 seconds).

### Test Step:
1. Start the user interface and do not interact with it.
2. Wait for the specified timeout duration.

### Expected Result:
- The GUI automatically closes after the timeout period (e.g., 300 seconds).
- No selection is made, and the system gracefully handles the timeout.

### UI Display
<img src="/home/af/ros2_ws/src/sf_master/src/sf_ui/resource/Ui-Dis_output.png" alt="drawing" width="500"/>