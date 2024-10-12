import unittest
from unittest.mock import MagicMock
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sf_msgs.msg import EgoPosition, ParkingSpotContainer, SelectedParking, Point
from sf_msgs.srv import SelectSpot, BookSpot
from pathlib import Path
from .A_STAR import parse_map_data, astar  # Assuming A_STAR module is in the same directory as PathPlan

# Import the PathPlan class from your script
from your_module import PathPlan  # Replace 'your_module' with your actual module name

class TestPathPlan(unittest.TestCase):

    def setUp(self):
        # Mocking necessary components
        self.node = Node("test_node")
        self.path_plan = PathPlan()
        self.path_plan.get_logger().info = MagicMock()  # Mocking logger info method
        self.path_plan.create_subscription = MagicMock()
        self.path_plan.create_publisher = MagicMock()
        self.path_plan.create_timer = MagicMock()
        self.path_plan.create_client = MagicMock()

    def test_egoposition_callback(self):
        ego_position_msg = EgoPosition()
        self.path_plan.egoposition_callback(ego_position_msg)
        self.assertEqual(self.path_plan.ego_position, ego_position_msg)

    def test_parkingspots_callback_with_availability(self):
        parking_spot_msg = ParkingSpotContainer()
        available_parking_spot = parking_spot_msg.available_parking_spots.add()
        available_parking_spot.status = 0  # Available parking spot
        self.path_plan.parkingspots_callback(parking_spot_msg)
        self.assertTrue(self.path_plan.availability)

    def test_parkingspots_callback_without_availability(self):
        parking_spot_msg = ParkingSpotContainer()
        self.path_plan.parkingspots_callback(parking_spot_msg)
        self.assertFalse(self.path_plan.availability)

    def test_calculate_distance(self):
        edges = [(0, 1, 10), (1, 2, 5), (2, 3, 8)]
        paths = [0, 1, 2, 3]
        expected_distance = 23.0  # Total distance based on given edges and paths
        distance = self.path_plan.calculate_distance(edges, paths)
        self.assertEqual(distance, expected_distance)

    def test_select_parking_spot_no_availability(self):
        self.path_plan.availability = False
        self.path_plan.select_parking_spot()
        self.path_plan.get_logger().info.assert_called_with("no parking spot available")

    def test_select_parking_spot_no_ego_position(self):
        self.path_plan.availability = True
        self.path_plan.ego_position = None
        self.path_plan.select_parking_spot()
        self.path_plan.get_logger().info.assert_called_with("ego position not available")

    def test_select_parking_spot_with_availability_and_position(self):
        self.path_plan.availability = True
        self.path_plan.ego_position = EgoPosition(x_coordinate=1.0, y_coordinate=2.0)
        self.path_plan.parking_spots = ParkingSpotContainer()
        parking_spot = self.path_plan.parking_spots.available_parking_spots.add()
        parking_spot.status = 0  # Available parking spot
        parking_spot.id = 1
        parking_spot.x = 3.0
        parking_spot.y = 4.0
        self.path_plan.select_client.call_async = MagicMock()
        self.path_plan.select_parking_spot()
        self.assertTrue(self.path_plan.book_timer.cancelled)
        self.path_plan.publish_path()
        self.path_plan.publish_final_parking_point()
        self.path_plan.select_client.call_async.assert_called_once()

    def test_book_nearest_parking_spot_success(self):
        self.path_plan.selected_spot_id = 1
        self.path_plan.book_request.spot_id = 1
        self.path_plan.book_client.call_async = MagicMock()
        self.path_plan.book_nearest_parking_spot(1)
        self.path_plan.get_logger().info.assert_called_with("spot booked")

    def test_book_nearest_parking_spot_failure(self):
        self.path_plan.selected_spot_id = 1
        self.path_plan.book_request.spot_id = 1
        self.path_plan.book_client.call_async = MagicMock()
        self.path_plan.book_client.call_async.return_value.result.return_value.booked = False
        self.path_plan.book_nearest_parking_spot(1)
        self.path_plan.get_logger().info.assert_called_with("spot not booked")

    def test_publish_path(self):
        self.path_plan.selected_spot_id = 1
        self.path_plan.availabile_path_dict = {1: [0, 1, 2]}
        self.path_plan.nodes = {0: PoseStamped(x=0.0, y=0.0), 1: PoseStamped(x=1.0, y=1.0), 2: PoseStamped(x=2.0, y=2.0)}
        self.path_plan.publish_path()
        self.path_plan.path_pub.publish.assert_called_once()

    def test_publish_final_parking_point(self):
        self.path_plan.last_point = (1.0, 2.0)
        self.path_plan.publish_final_parking_point()
        self.path_plan.selected_parking_publisher.publish.assert_called_once()

if __name__ == '__main__':
    unittest.main()

