import unittest
import rclpy
from rclpy.node import Node
from unittest.mock import MagicMock

from sf_msgs.msg import EgoPosition, ParkingSpot, ParkingSpotContainer, RouteData, Point

from sf_path_plan.sf_route_plan import sf_route 

class TestRoutePlan(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.route_planning = RoutePlan()
    
    
    def tearDown(self):
        self.route_planning.destroy_node()
        rclpy.shutdown()

    # TC_01: test if the Node can being created
    def test_init_node(self):

        assert self.route_planning.get_logger().info("route planning is running")    

    def test_egoposition_callback(self):
        # Test when ego position is received
        
        self.route_planning.egoposition = None
        assert self.route_planning.get_logger().info("TC_02.1: Test when ego position is received")
        

    def test_parkingspots_callback(self):
        # Test when parking spots are received
        self.route_planning = RoutePlan()
        parking_spot_msg = ParkingSpotContainer()
        parking_spot = ParkingSpot(id=1, x=0, y=0, status=0)
        parking_spot_msg.available_parking_spots.append(parking_spot)
        self.route_planning.parkingspots_callback(parking_spot_msg)
        assert self.route_planning.availability.get_logger().info("TC.02.2: Test when parking spots are received")

        # Test when no parking spots are available
        parking_spot_msg.available_parking_spots.clear()
        self.route_planning.parkingspots_callback(parking_spot_msg)
        self.assertFalse(route_planning.availability)

    def test_select_parking_spot_service(self):
        # Test select_parking_spot service call
        self.route_planning = RoutePlan()
        self.route_planning.get_logger().info("TC.02.3: Test select_parking_spot service call")
        self.route_planning.select_client.call_async = MagicMock()
        self.route_planning.select_request = SelectSpot.Request()
        self.route_planning.select_parking_spot()
        self.route_planning.select_client.call_async.assert_called_once_with(self.route_planning.select_request)

    def test_book_parking_spot_service(self):
        # Test book_parking_spot service call
        self.route_planning = rp.RoutePlan()
        self.route_planning.get_logger().info("TC.02.4: Test book_parking_spot service call")
        spot_id = 1
        self.route_planning.book_request = BookSpot.Request()
        self.route_planning.book_request.spot_id = spot_id
        self.route_planning.book_client.call_async = MagicMock()
        self.route_planning.book_nearest_parking_spot(spot_id)
        self.route_planning.book_client.call_async.assert_called_once_with(self.route_planning.book_request)    

    def test_calculate_distance(self):
        # Test distance calculation
        self.route_planning = rp.RoutePlan()
        self.route_planning.get_logger().info("TC.02.5: Test distance calculation")
        point1 = (0, 0)    # to be change
        point2 = (3, 4)    # to be change
        expected_distance = 5   # to be change
        distance = self.route_planning.calculate_distance(point1, point2)
        self.assertEqual(distance, expected_distance)

    def test_select_parking_spot(self):
        # Test when no parking spots are available
        self.route_planning = rp.RoutePlan()
        self.route_planning.get_logger().info("TC.02.6: Test when no parking spots are available")
        self.route_planning.availability = False
        self.route_planning.select_parking_spot()
        self.assertFalse(self.route_planning.select_client.call_async.called)  

        # Test when ego position is not available
        self.route_planning.get_logger().info("TC.02.7: Test when ego position is not available")
        self.route_planning.availability = True
        self.route_planning.ego_position = None
        self.route_planning.select_parking_spot()
        self.assertFalse(self.route_planning.select_client.call_async.called)  

        # Test when parking spots and ego position are available
        self.route_planning.get_logger().info("TC.02.8: Test when parking spots and ego position are available")
        self.route_planning.ego_position = EgoPosition(x_coordinate=0, y_coordinate=0)   # to be change according to coordinate of vehicle
        parking_spot_msg = ParkingSpotContainer()
        parking_spot_msg.available_parking_spots.append(ParkingSpot(id=1, x=1, y=1, status=0))  # to be change acc. to coordinate of spot
        self.route_planning.parkingspots_callback(parking_spot_msg)
        self.route_planning.select_client.call_async = MagicMock()
        self.route_planning.select_parking_spot()
        self.route_planning.select_client.call_async.assert_called_once_with(self.route_planning.select_request)  

    def test_book_nearest_parking_spot(self):
        # Test booking nearest parking spot when successfully booked
        self.route_planning = rp.RoutePlan()
        self.route_planning.get_logger().info("TC.02.9: Test booking nearest parking spot when successfully booked")
        spot_id = 1
        self.route_planning.book_client.call_async = MagicMock()
        self.route_planning.book_nearest_parking_spot(spot_id)
        self.route_planning.book_client.call_async.assert_called_once_with(self.route_planning.book_request)

        # Test booking nearest parking spot when not successfully booked
        self.route_planning.get_logger().info("TC.02.10: Test booking nearest parking spot when not successfully booked")
        self.route_planning.book_client.call_async = MagicMock()
        self.route_planning.book_request.booked = False
        self.route_planning.book_nearest_parking_spot(spot_id)
        self.route_planning.book_client.call_async.assert_called_once_with(self.route_planning.book_request)
        self.assertFalse(self.route_planning.book_timer.is_canceled())

    def test_calculate_route(self):
        # Test calculating route when parking spot is found
        self.route_planning = rp.RoutePlan()
        self.route_planning.get_logger().info("TC.02.11: Test calculating route when parking spot is found")
        spot_id = 1
        parking_spot_msg = ParkingSpotContainer()
        parking_spot_msg.available_parking_spots.append(ParkingSpot(id=spot_id, x=2, y=2, status=0)) # to be change 
        self.route_planning.parkingspots_callback(parking_spot_msg)
        self.route_planning.calculate_route(spot_id)
        self.assertEqual(len(self.route_planning.route_data.points), 1)
        self.assertEqual(self.route_planning.route_data.points[0].x, 1.25)  
        self.assertEqual(self.route_planning.route_data.points[0].y, 2)      
        
    def test_publish_route_data(self):
        # Test publishing route data
        self.route_planning = rp.RoutePlan()
        self.route_planning.get_logger().info("TC.02.12: Test publishing route data")
        route_data = RouteData()
        self.route_planning.route_publisher.publish = MagicMock()
        self.route_planning.publish_route_data()
        self.route_planning.route_publisher.publish.assert_called_once_with(route_data)

if __name__ == '__main__':
    unittest.main()
