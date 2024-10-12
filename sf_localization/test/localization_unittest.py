import rclpy
import unittest
from sf_localization import sf_ego_localization as el
from sf_msgs.msg import EgoPosition
from mocap_msgs.msg import RigidBodies, RigidBody
import time                                     
# import math                                     
# import pymap3d as pm


class TestSfLocalization(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.localization_class = el.Localization()

    def tearDown(self):
        self.localization_class.destroy_node()
        rclpy.shutdown()

    def test_euler_from_quaternion(self):
        
        # Test euler_from_quaternion method with known quaternion values
        roll, pitch, yaw = self.localization_class.euler_from_quaternion(0.0, 0.0, 0.0, 1.0)
        assert roll == 0.0
        assert pitch == 0.0
        assert yaw == 0.0

        

    def test_rigid_bodies_callback(self):    
        

        #rigid_bodies_msg
        rigid_bodies_msg = RigidBodies()
        rigied_bodies = []
        
        rb = RigidBody()

        rb.rigid_body_name = "7"  # Setting rigid body name to match vehicle_id
        rb.pose.position.x = 10.0  # Setting x position
        rb.pose.position.y = 20.0  # Setting y position
        q_x  = rb.pose.orientation.x = 0.0
        q_y  = rb.pose.orientation.y = 0.0
        q_z  = rb.pose.orientation.z = 0.0
        q_w  = rb.pose.orientation.w = 1.0

        
        rigied_bodies.append(rb)
        rigid_bodies_msg.rigidbodies = rigied_bodies

        # Call rigid_bodies_callback
        self.localization_class.rigid_bodies_callback(rigid_bodies_msg)

        # Check if EgoPosition message is published
        #assert ego_position_msg is not None  # Assert that a message has been published

        ego_position_msg = EgoPosition()  # Get the published message
        # Check the published EgoPosition message contains the correct values
        #assert ego_position_msg.x_coordinate ==   # Check x coordinate
        #assert ego_position_msg.y_coordinate == rb.pose.position.y  # Check y coordinate

        roll, pitch, yaw = self.localization_class.euler_from_quaternion(q_x, q_y, q_z, q_w)

        #add assert for  yaw  
        # expected_yaw = 0.0  # Expected yaw angle in degrees
        # assert yaw == expected_yaw

        # add conversation of cordinates
        
        

    def test_acceleration_values(self):

        
        # Test if acceleration_values list is populated correctly
        rigid_bodies_msg = RigidBodies()
        # Create a rigid body message with necessary fields populated

        # Test if acceleration_values list is populated correctly under different conditions

        # Case 1: No previous position set
        #rigid_bodies_msg = RigidBodies()
        rigid_body = RigidBody()
        rigid_body.rigid_body_name = "7"
        rigid_body.pose.position.x = 1.0
        rigid_body.pose.position.y = 2.0
        rigid_body.pose.orientation.x = 0.0
        rigid_body.pose.orientation.y = 0.0
        rigid_body.pose.orientation.z = 0.0
        rigid_body.pose.orientation.w = 1.0
        rigid_bodies_msg.rigidbodies.append(rigid_body)

        self.localization_class.rigid_bodies_callback(rigid_bodies_msg)
        # Assuming that the callback method will not populate acceleration_values list if no previous position is set
        assert len(self.localization_class.acceleration_values) == 0

        # Case 2: Zero time difference
        # Assuming that the previous position is set and the time difference is zero
        # rigid_bodies_msg = RigidBodies()
        # rigid_body = RigidBodies.RigidBody()
        # rigid_body.rigid_body_name = "7"
        # rigid_body.pose.position.x = 2.0
        # rigid_body.pose.position.y = 3.0

        # rigid_bodies_msg.rigidbodies.append(rigid_body)

        self.localization_class.prev_pos = (1.0, 2.0, round(time.time() * 1000))  # Assuming previous position is set
        self.localization_class.rigid_bodies_callback(rigid_bodies_msg)
        # Assuming that the callback method will not populate acceleration_values list if time difference is zero
        assert len(self.localization_class.acceleration_values) == 0
        

        # Case 3: Valid acceleration calculation
        # Assuming valid position and time difference to calculate acceleration
        # rigid_bodies_msg = RigidBodies()
        # rigid_body = RigidBodies.RigidBody()
        # rigid_body.rigid_body_name = "7"
        # rigid_body.pose.position.x = 3.0
        # rigid_body.pose.position.y = 4.0
        # rigid_body.pose.orientation.x = 0.0
        # rigid_body.pose.orientation.y = 0.0
        # rigid_body.pose.orientation.z = 0.0
        # rigid_body.pose.orientation.w = 1.0
        # rigid_bodies_msg.rigidbodies.append(rigid_body)

        self.localization_class.prev_pos = (2.0, 3.0, round(time.time() * 1000) - 1000)  # Assuming previous position and time are set
        self.localization_class.rigid_bodies_callback(rigid_bodies_msg)
        # Assuming that the callback method will populate acceleration_values list with valid acceleration value
        assert len(self.localization_class.acceleration_values) > 0

        

if __name__ == '__main__':
    unittest.main()
