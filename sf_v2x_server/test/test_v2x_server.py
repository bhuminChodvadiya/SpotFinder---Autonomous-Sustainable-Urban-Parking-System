import unittest

import rclpy
import time

from mocap_msgs.msg import RigidBodies, RigidBody 
from sf_msgs.srv import BookSpot
from sf_v2x_server.sf_v2x_server import V2XServer

class TestV2xServer(unittest.TestCase):

    rb_msg = RigidBodies()
    rigid_bodies = []
    
    rigid_body = RigidBody()
    rigid_body.rigid_body_name = "7"
    rigid_body.pose.position.x = 4.75
    rigid_body.pose.position.y = 5.10
    rigid_bodies.append(rigid_body)

    rigid_body = RigidBody()
    rigid_body.rigid_body_name = "10"
    rigid_body.pose.position.x = 4.10
    rigid_body.pose.position.y = 5.90
    rigid_bodies.append(rigid_body)
    rb_msg.rigidbodies = rigid_bodies

    def setUp(self):
        rclpy.init()
        self.v2x_server = V2XServer()

    def tearDown(self):
        self.v2x_server.destroy_node()
        rclpy.shutdown()    

    # TC_1: Test if rigid_bodies_callback is called and the data correctly stored
    def test_rigid_bodies_callback(self):

        self.v2x_server.rigid_bodies_callback(self.rb_msg)
        assert len(self.v2x_server.rigid_bodies_msg.rigidbodies) == 2
        assert self.v2x_server.rigid_bodies_msg.rigidbodies[0].rigid_body_name == "7"
        assert self.v2x_server.rigid_bodies_msg.rigidbodies[1].rigid_body_name == "10"

    # TC_2: Test if spot can be booked
    def test_check_if_booked(self):

        # no parking spots are booked
        assert self.v2x_server.check_if_booked(0) == False

        # requestes SPot is already Booked
        self.v2x_server.booked_parking_spots = [(0, time.time())]
        assert self.v2x_server.check_if_booked(0) == True

        # booking duration of requestion parking spot is over
        self.v2x_server.booked_parking_spots = [(0, time.time()-60)]
        assert self.v2x_server.check_if_booked(0) == False

    # TC_3: Test if parking spots are published correctly
    def test_parking_Spot_callback(self):

        self.v2x_server.rigid_bodies_msg = self.rb_msg
        self.v2x_server.parking_spot_callback()

        # total amount of parking spots
        assert self.v2x_server.evcsn.evcsn.evcsn_data.total_number_of_stations == 4
        # spot 1
        assert self.v2x_server.evcsn.evcsn.evcsn_data.charging_stations_data[0].charging_spots_available[0].parking_places_data[0].blocking == True
        # spot 1
        assert self.v2x_server.evcsn.evcsn.evcsn_data.charging_stations_data[1].charging_spots_available[0].parking_places_data[0].blocking == True
        # spot 1
        assert self.v2x_server.evcsn.evcsn.evcsn_data.charging_stations_data[2].charging_spots_available[0].parking_places_data[0].blocking == False
        # spot 1
        assert self.v2x_server.evcsn.evcsn.evcsn_data.charging_stations_data[3].charging_spots_available[0].parking_places_data[0].blocking == False


    # TC_4: Test  book_parking_spot() not occupied and not booked
    def test_book_parking_spot_not_occupied_not_booked(self):
        self.v2x_server.rigid_bodies_msg = self.rb_msg
        
        request = BookSpot.Request()
        reponse = BookSpot.Response()
        request.spot_id = 0 

        self.v2x_server.book_parking_spot(request,reponse)

        assert self.v2x_server.get_logger().info("spot with id: 0 is booked")

    # TC_5: Test  book_parking_spot() not occupied and but booked
    def test_book_parking_spot_not_occupied_booked(self):
        self.v2x_server.rigid_bodies_msg = self.rb_msg
        self.v2x_server.booked_parking_spots = [(3, time.time())]

        request = BookSpot.Request()
        reponse = BookSpot.Response()
        request.spot_id = 3 

        self.v2x_server.book_parking_spot(request,reponse)

        assert self.v2x_server.get_logger().info("spot with id: 3 is not booked")

if __name__ == '__main__':
    unittest.main()