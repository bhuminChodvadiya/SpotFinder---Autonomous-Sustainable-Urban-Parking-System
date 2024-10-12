import unittest
from unittest.mock import MagicMock
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sf_msgs.msg import EgoPosition
from  sf_driving_controller.parking_manoeuver import ParkingManeuverNode

class TestParkingManeuverNode(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.parking_manoeuver = ParkingManeuverNode()

    def test_initial_state(self):
        """
        Test ID: T001
        Description: Verify the initial state of the ParkingManeuverNode.
        """
        self.assertIsNone(self.parking_manoeuver.ego_position)
        self.assertFalse(self.parking_manoeuver.parking_in_progress)
        self.assertIsNone(self.parking_manoeuver.initial_position)
        self.assertEqual(self.parking_manoeuver.total_distance_moved, 0.0)
        self.assertEqual(self.parking_manoeuver.forward_distance_moved, 0.0)
        self.assertEqual(self.parking_manoeuver.backward_distance_moved, 0.0)
        self.assertTrue(self.parking_manoeuver.aligning_with_spot)
        self.assertFalse(self.parking_manoeuver.reversing_into_spot)

    def test_start_parking(self):
        """
        Test ID: T002
        Description: Ensure start_parking method initializes the parking process correctly.
        """
        self.parking_manoeuver.start_parking()
        self.assertTrue(self.parking_manoeuver.parking_in_progress)
        self.assertIsNone(self.parking_manoeuver.initial_position)
        self.assertEqual(self.parking_manoeuver.total_distance_moved, 0.0)
        self.assertEqual(self.parking_manoeuver.forward_distance_moved, 0.0)
        self.assertEqual(self.parking_manoeuver.backward_distance_moved, 0.0)
        self.assertTrue(self.parking_manoeuver.aligning_with_spot)
        self.assertFalse(self.parking_manoeuver.reversing_into_spot)

    def test_position_callback(self):
        """
        Test ID: T003
        Description: Ensure position_callback method updates the ego position correctly.
        """
        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = 1.0
        ego_position_msg.y_coordinate = 2.0
        ego_position_msg.yaw_angle = math.radians(30)

        self.parking_manoeuver.position_callback(ego_position_msg)
        self.assertEqual(self.parking_manoeuver.ego_position.x_coordinate, 1.0)
        self.assertEqual(self.parking_manoeuver.ego_position.y_coordinate, 2.0)
        self.assertEqual(self.parking_manoeuver.ego_position.yaw_angle, math.radians(30))

    def test_calculate_steering_angle(self):
        """
        Test ID: T004
        Description: Ensure calculate_steering_angle method calculates the correct steering angle.
        """
        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = 1.0
        ego_position_msg.y_coordinate = 2.0
        ego_position_msg.yaw_angle = 0.0  # Facing along the x-axis

        self.parking_manoeuver.ego_position = ego_position_msg
        self.parking_manoeuver.parking_spot = {
            "x_coordinate": 1.0,
            "y_coordinate": 3.0  # Directly above the car
        }

        # For reverse=False
        angle = self.parking_manoeuver.calculate_steering_angle(reverse=False)
        self.assertAlmostEqual(angle, 90.0, delta=1.0)

        # For reverse=True
        angle = self.parking_manoeuver.calculate_steering_angle(reverse=True)
        self.assertAlmostEqual(angle, -90.0, delta=1.0)

    def test_calculate_steering_angle_small_angle(self):
        """
        Test ID: T005
        Description: Test calculate_steering_angle for small angle changes.
        """
        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = 1.0
        ego_position_msg.y_coordinate = 2.0
        ego_position_msg.yaw_angle = math.radians(5)  # Small yaw angle

        self.parking_manoeuver.ego_position = ego_position_msg
        self.parking_manoeuver.parking_spot = {
            "x_coordinate": 1.1,
            "y_coordinate": 2.1  # Close to the current position
        }

        angle = self.parking_manoeuver.calculate_steering_angle(reverse=False)
        self.assertAlmostEqual(angle, 45.0, delta=1.0)

    def test_publish_twist_message(self):
        """
        Test ID: T006
        Description: Ensure publish_twist_message method publishes the correct Twist message.
        """
        self.parking_manoeuver.cmd_publisher.publish = MagicMock()
        self.parking_manoeuver.twist_msg.linear.x = 0.5
        self.parking_manoeuver.twist_msg.angular.z = 0.1

        self.parking_manoeuver.publish_twist_message()
        self.parking_manoeuver.cmd_publisher.publish.assert_called_once_with(self.parking_manoeuver.twist_msg)

    def test_parking_maneuver_alignment_phase(self):
        """
        Test ID: T007
        Description: Test parking_maneuver method during the alignment phase.
        """
        self.parking_manoeuver.start_parking()
        self.parking_manoeuver.aligning_with_spot = True
        self.parking_manoeuver.reversing_into_spot = False

        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = 1.0
        ego_position_msg.y_coordinate = 4.5
        ego_position_msg.yaw_angle = 0.0

        self.parking_manoeuver.ego_position = ego_position_msg
        self.parking_manoeuver.parking_maneuver()

        self.assertEqual(self.parking_manoeuver.twist_msg.linear.x, 0.2)

    def test_parking_maneuver_reversing_phase(self):
        """
        Test ID: T008
        Description: Test parking_maneuver method during the reversing phase.
        """
        self.parking_manoeuver.start_parking()
        self.parking_manoeuver.aligning_with_spot = False
        self.parking_manoeuver.reversing_into_spot = True

        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = 4.5
        ego_position_msg.y_coordinate = 5.0
        ego_position_msg.yaw_angle = 0.0

        self.parking_manoeuver.ego_position = ego_position_msg
        self.parking_manoeuver.parking_maneuver()

        self.assertEqual(self.parking_manoeuver.twist_msg.linear.x, -0.2)

    def test_boundary_conditions(self):
        """
        Test ID: T009
        Description: Test parking_maneuver when the vehicle is exactly at the boundary of the parking spot.
        """
        self.parking_manoeuver.start_parking()
        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = self.parking_manoeuver.parking_spot["x_coordinate"]
        ego_position_msg.y_coordinate = self.parking_manoeuver.parking_spot["y_coordinate"]
        ego_position_msg.yaw_angle = 0.0
        self.parking_manoeuver.ego_position = ego_position_msg

        self.parking_manoeuver.parking_maneuver()

        self.assertEqual(self.parking_manoeuver.twist_msg.linear.x, 0.0)
        self.assertFalse(self.parking_manoeuver.parking_in_progress)

    def test_invalid_ego_position(self):
        """
        Test ID: T010
        Description: Ensure the system handles null or invalid ego position data.
        """
        self.parking_manoeuver.start_parking()
        self.parking_manoeuver.ego_position = None
        self.parking_manoeuver.parking_maneuver()
        self.assertFalse(self.parking_manoeuver.parking_in_progress)

    def test_invalid_parking_spot(self):
        """
        Test ID: T011
        Description: Ensure the system handles invalid parking spot coordinates.
        """
        self.parking_manoeuver.start_parking()
        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = 1.0
        ego_position_msg.y_coordinate = 2.0
        ego_position_msg.yaw_angle = 0.0
        self.parking_manoeuver.ego_position = ego_position_msg

        self.parking_manoeuver.parking_spot["x_coordinate"] = None
        self.parking_manoeuver.parking_maneuver()

        self.assertEqual(self.parking_manoeuver.twist_msg.linear.x, 0.0)
        self.assertEqual(self.parking_manoeuver.twist_msg.angular.z, 0.0)

    def test_high_frequency_position_updates(self):
        """
        Test ID: T012
        Description: Test how the system handles rapid position updates.
        """
        self.parking_manoeuver.start_parking()
        for i in range(100):
            ego_position_msg = EgoPosition()
            ego_position_msg.x_coordinate = 1.0 + 0.01 * i
            ego_position_msg.y_coordinate = 2.0
            ego_position_msg.yaw_angle = 0.0
            self.parking_manoeuver.position_callback(ego_position_msg)
            self.parking_manoeuver.parking_maneuver()

        self.assertTrue(self.parking_manoeuver.parking_in_progress)

    def test_edge_case_within_parking_spot(self):
        """
        Test ID: T013
        Description: Test behavior if the vehicle is already within the parking spot when starting the parking maneuver.
        """
        self.parking_manoeuver.start_parking()
        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = self.parking_manoeuver.parking_spot["x_coordinate"]
        ego_position_msg.y_coordinate = self.parking_manoeuver.parking_spot["y_coordinate"]
        ego_position_msg.yaw_angle = 0.0
        self.parking_manoeuver.ego_position = ego_position_msg

        self.parking_manoeuver.parking_maneuver()

        self.assertFalse(self.parking_manoeuver.parking_in_progress)

    def test_small_changes_in_position(self):
        """
        Test ID: T014
        Description: Test how small changes in ego position affect the maneuvering logic.
        """
        self.parking_manoeuver.start_parking()
        self.parking_manoeuver.aligning_with_spot = True
        self.parking_manoeuver.reversing_into_spot = False

        initial_x = 1.0
        initial_y = 4.5
        initial_yaw = 0.0

        for i in range(10):
            ego_position_msg = EgoPosition()
            ego_position_msg.x_coordinate = initial_x + 0.01 * i
            ego_position_msg.y_coordinate = initial_y
            ego_position_msg.yaw_angle = initial_yaw
            self.parking_manoeuver.position_callback(ego_position_msg)
            self.parking_manoeuver.parking_maneuver()

        self.assertTrue(self.parking_manoeuver.parking_in_progress)


if __name__ == '__main__':
    unittest.main()
