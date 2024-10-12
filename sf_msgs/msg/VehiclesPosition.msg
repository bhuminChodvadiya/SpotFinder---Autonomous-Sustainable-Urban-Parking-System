# This message type is used for sending all the vehicle positions which are converted from their CAM message to model city co-ordinates

uint8 vehicle_id    # ID of the vehicle generating the CAM message

float32 x_coordinate        # x coordinate of the vehicle in model city coordinates

float32 y_coordinate        # y coordinate of the vehicle in model city coordinates

float32 yaw_angle           # yaw angle in degree of the vehicle