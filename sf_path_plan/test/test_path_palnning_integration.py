import rclpy
from rclpy.node import Node
from sf_msgs.msg import EgoPosition, ParkingSpotContainer, SelectedParking
from sf_msgs.srv import BookSpot
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import threading
import time

class TestIntegrationValidEgoParking(Node):
    def __init__(self):
        super().__init__('test_integration_valid_ego_parking')
        self.ego_subscriber = self.create_subscription(EgoPosition, '/ego_position', self.ego_position_callback, 10)
        self.parking_subscriber = self.create_subscription(ParkingSpotContainer, '/parking_spots', self.parking_spots_callback, 10)
        self.valid_ego_position_received = False
        self.valid_parking_spots_received = False

    def ego_position_callback(self, msg):
        self.get_logger().info('Received Ego Position: %s' % msg)
        self.valid_ego_position_received = True

    def parking_spots_callback(self, msg):
        self.get_logger().info('Received Parking Spots: %s' % msg)
        self.valid_parking_spots_received = True

    def run_test(self):
        timeout = 10  # seconds
        start_time = time.time()

        while not self.valid_ego_position_received or not self.valid_parking_spots_received:
            rclpy.spin_once(self, timeout_sec=1.0)
            if time.time() - start_time > timeout:
                self.get_logger().info('Timeout: Did not receive valid ego position and parking spots in time.')
                return False

        return True

class TestIntegrationBookedParkingSpot(Node):
    def __init__(self):
        super().__init__('test_integration_booked_parking_spot')
        self.book_client = self.create_client(BookSpot, 'book_parking_spot')
        self.book_request = BookSpot.Request()

    def call_book_service(self):
        while not self.book_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Book service not available, waiting...')
        self.book_request.spot_id = 1  # Replace with actual spot ID
        future = self.book_client.call_async(self.book_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().booked

    def run_test(self):
        booked = self.call_book_service()
        self.get_logger().info('Parking spot booked: %s' % booked)
        return booked

class TestIntegrationMultipleParkingSpots(Node):
    def __init__(self):
        super().__init__('test_integration_multiple_parking_spots')
        self.ego_subscriber = self.create_subscription(EgoPosition, '/ego_position', self.ego_position_callback, 10)
        self.parking_subscriber = self.create_subscription(ParkingSpotContainer, '/parking_spots', self.parking_spots_callback, 10)
        self.multiple_spots_received = False

    def ego_position_callback(self, msg):
        self.get_logger().info('Received Ego Position: %s' % msg)

    def parking_spots_callback(self, msg):
        self.get_logger().info('Received Parking Spots: %s' % msg)
        if len(msg.available_parking_spots) > 1:
            self.multiple_spots_received = True

    def run_test(self):
        timeout = 10  # seconds
        start_time = time.time()

        while not self.multiple_spots_received:
            rclpy.spin_once(self, timeout_sec=1.0)
            if time.time() - start_time > timeout:
                self.get_logger().info('Timeout: Did not receive multiple parking spots in time.')
                return False

        return True

class TestIntegrationConcurrentRequests(Node):
    def __init__(self):
        super().__init__('test_integration_concurrent_requests')
        self.ego_subscriber = self.create_subscription(EgoPosition, '/ego_position', self.ego_position_callback, 10)
        self.parking_subscriber = self.create_subscription(ParkingSpotContainer, '/parking_spots', self.parking_spots_callback, 10)
        self.concurrent_requests_handled = False
        self.lock = threading.Lock()

    def ego_position_callback(self, msg):
        self.get_logger().info('Received Ego Position: %s' % msg)
        self.handle_concurrent_requests()

    def parking_spots_callback(self, msg):
        self.get_logger().info('Received Parking Spots: %s' % msg)
        self.handle_concurrent_requests()

    def handle_concurrent_requests(self):
        with self.lock:
            if self.ego_subscriber.get_subscription_count() > 0 and self.parking_subscriber.get_subscription_count() > 0:
                self.concurrent_requests_handled = True

    def run_test(self):
        # Simulate concurrent updates
        ego_thread = threading.Thread(target=self.publish_ego_position)
        parking_thread = threading.Thread(target=self.publish_parking_spots)
        ego_thread.start()
        parking_thread.start()

        timeout = 10  # seconds
        start_time = time.time()

        while not self.concurrent_requests_handled:
            rclpy.spin_once(self, timeout_sec=1.0)
            if time.time() - start_time > timeout:
                self.get_logger().info('Timeout: Did not handle concurrent requests in time.')
                return False

        return True

    def publish_ego_position(self):
        ego_msg = EgoPosition()
        # Publish your ego position message here
        self.get_logger().info('Publishing Ego Position...')
        self.ego_subscriber.publish(ego_msg)

    def publish_parking_spots(self):
        parking_msg = ParkingSpotContainer()
        # Publish your parking spots message here
        self.get_logger().info('Publishing Parking Spots...')
        self.parking_subscriber.publish(parking_msg)

def main(args=None):
    rclpy.init(args=args)

    # Test Case IT-001: Integration Test for Valid Ego Position and Parking Spots
    test1 = TestIntegrationValidEgoParking()
    if test1.run_test():
        test1.get_logger().info('IT_PP_001 Passed')
    else:
        test1.get_logger().info('IT_PP_001 Failed')

    # Test Case IT-004: Integration Test for Booked Parking Spot
    test2 = TestIntegrationBookedParkingSpot()
    if test2.run_test():
        test2.get_logger().info('IT_PP_002 Passed')
    else:
        test2.get_logger().info('IT_PP_002 Failed')

    # Test Case IT-005: Integration Test for Multiple Available Parking Spots
    test3 = TestIntegrationMultipleParkingSpots()
    if test3.run_test():
        test3.get_logger().info('IT_PP_003 Passed')
    else:
        test3.get_logger().info('IT_PP_003 Failed')

    # Test Case IT-007: Integration Test for Concurrent Requests Handling
    test4 = TestIntegrationConcurrentRequests()
    if test4.run_test():
        test4.get_logger().info('IT_PP_004 Passed')
    else:
        test4.get_logger().info('IT_PP_004 Failed')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
