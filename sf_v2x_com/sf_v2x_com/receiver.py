import rclpy
from rclpy.node import Node
from v2x.msg import CAM ,EvcsnPdu
from sf_msgs.msg import VehiclesPosition, ParkingSpot, ParkingSpotContainer
import pymap3d as pm

class CamReceiverNode(Node):
    def __init__(self):
        super().__init__("cam_receiver")

        # Creating publisher with vehicle co-ordinates and yaw
        self.vehicles_positions_publisher = self.create_publisher(VehiclesPosition,
                                                                  "/all_vehicles_position", 10)
        self.spot_publisher = self.create_publisher(ParkingSpotContainer, "/parking_spots", 10)

        # Creating subscription for CAM messages, now it is getting these messages from the dummy CAM server
        self.create_subscription(CAM, '/common_topic_name', self.cam_callback, 10)
        self.create_subscription(EvcsnPdu, "/available_parking_spots", self.evscn_callback, 10)

        # Latitude and longitude values of origin of model city coordinates
        self.lat_0 = 50.24132213367954
        self.lon_0 = 11.321265180951718
        self.h_0 = 0

        # Initialize VehiclesPosition and ParkingSpotContainer
        self.v_pos = VehiclesPosition()
        self.parking_spot_container = ParkingSpotContainer()

        self.get_logger().info("CAM Receiver node is running...")

    def cam_callback(self, cam_msg):
        # Check for CAM messages generated only from other vehicles apart from the ego vehicle
        vehicle_ids = [9,10]

        # Checking for only vehicles apart from the ego vehicle
        if cam_msg.header.station_id in vehicle_ids:
            # Station id of the originating vehicle
            v_id = cam_msg.header.station_id
            # Latitude of the originating vehicle in WGS84
            v_lat = cam_msg.cam.cam_parameters.basic_container.reference_position.latitude / (10**7)
            # Longitude of the originating vehicle in WGS84
            v_lon = cam_msg.cam.cam_parameters.basic_container.reference_position.longitude / (10**7)
            # Heading of the originating vehicle
            v_yaw = cam_msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value / 10

            # Converting the negative yaw angles to range 0 to 360 as the received yaw angle range is -180 to 180
            if v_yaw > 180:
                v_yaw -= 360

            # the altitude has been set to zero since the altitude is not considered for any application
            v_alt = 0
            # Convert geodetic coordinates to ENU (model city) coordinates
            model_city_position = pm.geodetic2enu(v_lat, v_lon, v_alt, self.lat_0, self.lon_0, self.h_0)

            # Publish the vehicle positions with respect to model city coordinates
            self.v_pos = VehiclesPosition()
            self.v_pos.vehicle_id = v_id
            self.v_pos.x_coordinate = model_city_position[0]     # East is considered model city x-coordinates
            self.v_pos.y_coordinate = model_city_position[1]     # North is considered model city y-coordinates

            self.v_pos.yaw_angle = v_yaw      # Yaw angle of the vehicle

            #publishing other vehicles positions
            self.vehicles_positions_publisher.publish(self.v_pos)

    def evscn_callback(self, evcsn_msg):
        """
        EVSCN For Parking Spot Availibility
        """
        self.parking_spot_container = ParkingSpotContainer()
        available_parking_spots = []

        # Getting charging station data
        for charging_station in evcsn_msg.evcsn.evcsn_data.charging_stations_data:

            parking_spot = ParkingSpot()
            parking_spot.id = charging_station.charging_station_id

            # set refrence coordinates (manually observed at model city (0,0))
            lat_0=50.24132213367954
            lon_0=11.321265180951718
            h_0 = 0

            # calculate coordinates
            lat = float(charging_station.charging_station_location.latitude * 10 **-7)
            lon = float(charging_station.charging_station_location.longitude * 10 **-7)
            h = 0

            # Converte geodetic to ENU ( East-North-Up) coordinates
            x,y,z=pm.geodetic2enu(lat = lat, lon = lon, h=h , lat0=lat_0 , lon0 = lon_0 , h0 = h_0 )

            # set parking spot attribute
            parking_spot.x = x
            parking_spot.y = y
            parking_spot.status = int(charging_station.charging_spots_available[0].parking_places_data[0].blocking)
            available_parking_spots.append(parking_spot)

        # Updating parking Spot container and publish
        self.parking_spot_container.available_parking_spots = available_parking_spots
        self.spot_publisher.publish(self.parking_spot_container)

def main(args=None):
    rclpy.init(args=args)
    node = CamReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()