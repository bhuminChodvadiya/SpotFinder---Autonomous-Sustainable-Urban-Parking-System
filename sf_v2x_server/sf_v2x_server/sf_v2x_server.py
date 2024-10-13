"""
Module for publishing EVCSN (Electric Vehicle Charging Station Network) messages 
and providing a parking spot booking functionality.
"""

import time
import json
import os
import rclpy
from rclpy.node import Node
import pymap3d as pm
from v2x.msg import EvcsnPdu, ItsChargingStationData, ItsChargingSpotDataElements, SpotAvailability
from mocap_msgs.msg import RigidBodies     # Mocap masg for subscriptions of vehicle position
from sf_msgs.srv import BookSpot

class V2XServer(Node):
    """
    This node is simulating a v2x by publishing evcn messages and 
    provide  a booking functionality
    """

    def __init__(self):
        super().__init__("v2x_server")

        # Initialize parameter
        parking_spots_file = 'parking_spots.json'
        self.spot_tolerance_x = 0.25
        self.spot_tolerance_y = 0.15
        self.lat_0 = 50.24132213367954
        self.lon_0 = 11.321265180951718
        self.h_0 = 0

        # Construct the path to the parking spots file
        parking_spots_path = os.path.join(os.path.dirname(__file__), parking_spots_file)

        # Check if the parking spots file exists
        if not os.path.exists(parking_spots_path):
            self.get_logger().error(f"parking spots file not found: {parking_spots_path}")
            raise FileNotFoundError(f"Parking spots file not found: {parking_spots_path}")
        
        # Initialize a timer to publish parking spot data periodically
        timer_period = 1
        self.parking_spot_timer = self.create_timer(
            callback=self.parking_spot_callback, timer_period_sec=timer_period)

        # publisher init
        self.parking_spot_publisher = self.create_publisher(
            EvcsnPdu, "/available_parking_spots", 10)

        # subscriber init
        self.create_subscription(RigidBodies, "/pose_modelcars", self.rigid_bodies_callback, 10)

        # services init
        self.create_service(BookSpot, 'book_parking_spot', self.book_parking_spot)

        # Load parking spots data from the JSON file
        with open(parking_spots_path, "r") as parking_spots_f:
            self.parking_spots = json.load(parking_spots_f)

        # init variables
        self.rigid_bodies_msg = None # Store the latest received vehicle positions
        self.booked_parking_spots = [] # List of booked parking spots (spot_id, timestamp of booking time)
        self.max_booking_time = 60 # Maximum time a parking spot can be booked in seconds
        self.evcsn = None # Variable to store the EVCSN message

        self.get_logger().info("V2X-Server Node is running ...")

    # publish all parking spots a as EVSN Message
    def parking_spot_callback(self):
        """
        this function is publishing a evcsn message containg all frour parking
        spots of the model city.
        """
        self.evcsn = EvcsnPdu()
        self.evcsn.evcsn.evcsn_data.total_number_of_stations = len(self.parking_spots)

        charging_station_data = []

        # iterate over all parkign spots
        for parking_spot in self.parking_spots:
            spot_x_coordinate = float(parking_spot.get("x_coordinte"))
            spot_y_coordinate = float(parking_spot.get("y_coordinte"))

            its_cs_data = ItsChargingStationData()

            its_cs_data.charging_station_id = parking_spot.get("id")

            # convert local to global coordinate system
            global_position = pm.enu2geodetic(e= spot_x_coordinate, n= spot_y_coordinate, u=0, lat0=self.lat_0, lon0=self.lon_0, h0=self.h_0)

            its_cs_data.charging_station_location.latitude = int(global_position[0] * (10 **7))
            its_cs_data.charging_station_location.longitude = int(global_position[1] * (10 **7))
            its_cs_data.charging_station_location.altitude.altitude_value = int(global_position[2] * 10 **7)
            its_cs_data.charging_station_location.altitude.altitude_confidence = 0

            its_charing_spot = ItsChargingSpotDataElements()
            spot_availability = SpotAvailability()

            # set status of parking spot
            if self.rigid_bodies_msg is not None:
                # for each vehicle in the Model city
                for rigid_body in self.rigid_bodies_msg.rigidbodies:
                    car_x = rigid_body.pose.position.x
                    car_y = rigid_body.pose.position.y
                    # if vehicle position and parking spot position overlapping, or parking spot is booked
                    if (((car_x <= spot_x_coordinate + self.spot_tolerance_x and car_x >= spot_x_coordinate - self.spot_tolerance_x ) and
                        (car_y <= spot_y_coordinate + self.spot_tolerance_y and car_y >= spot_y_coordinate - self.spot_tolerance_y )) or
                        (self.check_if_booked(parking_spot.get("id")) is True)):
                        # set parking spot to occupied
                        spot_availability.blocking = True
                        parking_spot["status"] = 1
                        break
                    else:
                    # set parking spot to not occupied
                        spot_availability.blocking = False
                        parking_spot["status"] = 0
            else:
                spot_availability.blocking = False
                parking_spot["status"] = 0

            its_charing_spot.parking_places_data= [spot_availability]
            its_cs_data.charging_spots_available = [its_charing_spot]
            charging_station_data.append(its_cs_data)
        self.evcsn.evcsn.evcsn_data.charging_stations_data = charging_station_data

        self.parking_spot_publisher.publish(self.evcsn)

    def rigid_bodies_callback(self, rigid_bodies_msg:RigidBodies):
        """
        stores the latest rigid_bodies msgs in a global variable
        """
        self.rigid_bodies_msg = rigid_bodies_msg

    def check_if_booked(self, parking_spot_id):
        """
        checks if a parking spot with a specific id is booked
        """
        booked = False
        for booked_parking_spot in self.booked_parking_spots:

            # if the max book duration reached, remove spot from booking list
            if time.time() - booked_parking_spot[1] > self.max_booking_time:
                self.booked_parking_spots.remove(booked_parking_spot)

            # if spot is already booked set booked to true
            elif booked_parking_spot[0] == parking_spot_id:
                booked = True
        return booked

    def book_parking_spot(self, request, response):
        """
        book the parking spot with a specific id
        """
        parking_spot_id = request.spot_id
        response = BookSpot.Response()

        occupied = None
        # indicates if the parking spot is already booked
        already_booked = self.check_if_booked(parking_spot_id)

        # check if the parking spot is accupied
        for parking_spot in self.parking_spots:
            if parking_spot.get("id") == parking_spot_id:
                occupied = bool(parking_spot.get("status"))
                break

        # if spot is not occupied and not booked then book parking spot
        if occupied is False and already_booked is False:
            response.booked = True
            self.booked_parking_spots.append((parking_spot_id, round(time.time())))
            self.get_logger().info("spot with id: "+str(parking_spot_id)+" is booked")
        else:
            response.booked = False
            self.get_logger().info("spot with id: "+str(parking_spot_id)+" is not booked")
            response.booked = False
        return response

def main(args=None):
    """
    main function for starting the node
    """
    rclpy.init(args=args)
    node = V2XServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
