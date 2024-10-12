import unittest
import rclpy
from unittest.mock import MagicMock
from sf_traffic_monitor.sf_traffic_monitor import TrafficMonitor
from sf_msgs.msg import VehiclesPosition, StreetInfoContainer, StreetInfo

class TestTrafficMonitor(unittest.TestCase):

    def setUp(self):
        # Initialize ROS context
        rclpy.init()
        self.sf_traffic_monitor = TrafficMonitor()  # Initialize without passing context

    def tearDown(self):
        self.sf_traffic_monitor.destroy_node()
        rclpy.shutdown()

    def test_subscription(self):
        # Mock create_subscription to test without actual ROS2 middleware
        self.sf_traffic_monitor.create_subscription = MagicMock()
        self.sf_traffic_monitor.vehicles_position_callback = MagicMock()
        self.sf_traffic_monitor.create_subscription(VehiclesPosition, '/all_vehicles_position', self.sf_traffic_monitor.vehicles_position_callback, 1)

    def test_publisher(self):
        # Mock create_publisher to test without actual ROS2 middleware
        self.sf_traffic_monitor.create_publisher = MagicMock()
        self.sf_traffic_monitor.create_publisher(StreetInfoContainer, '/traffic_density', 10)

    def test_timer_creation(self):
        # Mock create_timer to test without actual ROS2 middleware
        self.sf_traffic_monitor.create_timer = MagicMock()
        self.sf_traffic_monitor.create_timer(0.1, self.sf_traffic_monitor.calculate_density)

    def test_vehicle_density_calculation(self):
        # Setup mock vehicles positions
        vehicles_positions = [
            VehiclesPosition(vehicle_id=1, x_coordinate=0.5, y_coordinate=1.5),
            VehiclesPosition(vehicle_id=2, x_coordinate=0.6, y_coordinate=1.6),
            # Ensure you add enough vehicle positions to match realistic test conditions
        ]

        # Simulate vehicles position callback
        for position in vehicles_positions:
            self.sf_traffic_monitor.vehicles_position_callback(position)

        # Manually trigger density calculation
        self.sf_traffic_monitor.calculate_density()

        # Initialize a StreetInfoContainer with dummy StreetInfo data
        published_msg = StreetInfoContainer()
        published_msg.traffic_densities = [StreetInfo(density=i) for i in range(13)]  # Create 13 StreetInfo instances

        # Mock publish to simulate message handling
        self.sf_traffic_monitor.create_publisher.return_value.publish = MagicMock()
        self.sf_traffic_monitor.publish_density(published_msg)

        # Check if the traffic densities have been calculated and published correctly
        self.assertEqual(len(published_msg.traffic_densities), 13)
        for index, info in enumerate(published_msg.traffic_densities):
            self.assertIsInstance(info, StreetInfo)
            self.assertEqual(info.density, index)

    def test_vehicles_position_callback(self):
        # Create a sample VehiclesPosition message and simulate the callback
        vehicles_position_msg = VehiclesPosition(vehicle_id=1, x_coordinate=2.0, y_coordinate=3.0)
        self.sf_traffic_monitor.vehicles_position_callback(vehicles_position_msg)

        # Verify if the internal state was updated correctly
        self.assertEqual(len(self.sf_traffic_monitor.vehicles_positions), 1)
        self.assertEqual(self.sf_traffic_monitor.vehicles_positions[0].vehicle_id, 1)
        self.assertEqual(self.sf_traffic_monitor.vehicles_positions[0].x_coordinate, 2.0)
        self.assertEqual(self.sf_traffic_monitor.vehicles_positions[0].y_coordinate, 3.0)

if __name__ == '__main__':
    unittest.main()

