import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sf_msgs.srv import SelectSpot, BookSpot
from sf_msgs.msg import EgoPosition, ParkingSpotContainer, SelectedParking, Point, StreetInfoContainer, StreetInfo
from .A_STAR import parse_map_data, astar

class PathPlan(Node):
    def __init__(self):
        super().__init__("Path_planning")
        # Subscriptions to receive current ego vehicle position and available parking spots
        self.create_subscription(EgoPosition, '/ego_position', self.egoposition_callback, 10)
        self.create_subscription(ParkingSpotContainer, "/parking_spots", self.parkingspots_callback, 10)
        self.create_subscription(StreetInfoContainer, "/traffic_density", self.trafficmonitor_callback, 10)

        # # Publishers to publish final selected parking point(coordinate) and planned path
        self.selected_parking_publisher = self.create_publisher(SelectedParking, "/final_parking_point", 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        timer_period = 1  # Timer period for callbacks in seconds

        # Timers to periodically check for parking spots, publish path, and selected parking points
        self.book_timer = self.create_timer(timer_period, self.select_parking_spot)
        self.path_timer = self.create_timer(timer_period, self.publish_path)
        self.route_timer = self.create_timer(timer_period, self.publish_final_parking_point)

        # Callback groups to handle services in different threads
        select_cb_group = MutuallyExclusiveCallbackGroup()
        book_cb_group = MutuallyExclusiveCallbackGroup()

        self.last_point = (0.0, 0.0)
        self.final_path = []
        self.availabile_path_dict = {}
        self.last_point_dict = {}

        # Clients to interact with the Parking SelectSpot services
        self.select_client = self.create_client(SelectSpot, 'select_parking_spot', callback_group=select_cb_group)
        while not self.select_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('select service not available, waiting again...')
        self.select_request = SelectSpot.Request()

        # Clients to interact with the Parking BookSpot services
        self.book_client = self.create_client(BookSpot, 'book_parking_spot', callback_group=book_cb_group)
        while not self.book_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('book service not available, waiting again...')
        self.book_request = BookSpot.Request()

        # init variables
        self.ego_position = None
        self.parking_spots = None
        self.availability = False
        self.traffic_container = None
        
        # Initialize additional variables
        self.nodes = []
        self.edges = []
        self.start_position = (0, 0)
        self.end_position = (0, 0)
        self.future = None
        self.selected_spot_id = None
        self.last_point_x = 0
        self.last_point_y = 0

        self.get_logger().info("Path planning is running")

    def egoposition_callback(self, egoposition_msg):
        '''Callback for the ego vehicle's position.'''
        self.ego_position = egoposition_msg

    def trafficmonitor_callback(self, traffic_msg: StreetInfoContainer):
        '''Callback for the traffic data.'''
        self.traffic_container = traffic_msg.traffic_densities

        self.traffic_data = {}

        for street in traffic_msg.traffic_densities:
            self.traffic_data[street.street_name] = street.density
        

    def parkingspots_callback(self, parking_spot_msg):
        '''Callback for the list of available parking spots'''
        self.parking_spots = parking_spot_msg

        # Check for availability of any parking spots
        for available_parking_spot in parking_spot_msg.available_parking_spots:
            # checking if parking spot available( 0 = Available, 1 = Not available)
            if available_parking_spot.status == 0:  
                self.availability = True
                return
        self.availability= False

    def calculate_distance(self, edges, paths):
        '''Calculate the total distance of a given path using the edges.'''
        edge_dict = {}
        for start_node, end_node, cost, traffic_cost in edges:
            edge_dict[(start_node, end_node)] = cost + traffic_cost

        total_distance = 0
        for i in range(len(paths) - 1):
            start_node = paths[i]
            end_node = paths[i + 1]
            if (start_node, end_node) in edge_dict:
                total_distance += edge_dict[(start_node, end_node)]
            else:
                raise ValueError(f"No direct edge between {start_node} and {end_node}")
        return float(total_distance)

    def select_parking_spot(self):
        '''Select the nearest available parking spot using A* algorithm and book it'''
        if self.availability is False:   # checking if no parking spots are available
            self.get_logger().info("no parking spot available")
        elif self.ego_position is None:  # checking if ego position not availabe
            self.get_logger().info("ego position not available")
        elif self.traffic_container is None:  # checking if ego position not availabe
            self.get_logger().info("traffic data is not available")
        else:
            self.get_logger().info("select parking_spot")

            spots_with_distance = []

            # calculate distance to each availabel parking spot
            for parking_spot in self.parking_spots.available_parking_spots:
                print(parking_spot)
                # checking parking spot available( 0 = Available, 1 = Not available)
                if parking_spot.status == 0:
                    # Parse map data and set start and end positions for A* algorithm and the relative path for the map_node data
                    self.nodes, self.edges = parse_map_data('/home/af/ros2_ws/src/sf_master/src/sf_path_plan/map_nodedata/modelcitymap.txt', self.traffic_data)
                    # rel_path = 'src/sf_path_plan/map_nodedata/modelcitymap.txt'
                    # abs_path = os.path.join(os.path.dirname(__file__), rel_path)
                    # self.nodes, self.edges = parse_map_data(abs_path,self.traffic_data)
                    # ego position coordinates
                    self.start_position = (int(self.ego_position.x_coordinate * 100) , int(self.ego_position.y_coordinate * 100))
                    # parking spot coordinates
                    self.end_position = (int(parking_spot.x * 100), int(parking_spot.y* 100))  
                    # Run A* algorithm to find path for all availabel parking spots
                    available_path = astar(self.nodes, self.edges, self.start_position, self.end_position)
                    # Calculate distance of the path
                    distance = self.calculate_distance(self.edges, available_path)
                    parking_spot.distance = distance
                    self.availabile_path_dict[parking_spot.id] = available_path
                    self.last_point_dict[parking_spot.id] = (parking_spot.x, parking_spot.y)
                    spots_with_distance.append(parking_spot)

            # Sort spots by distance and select the closest ones
            spots_with_distance.sort(key=lambda x: x.distance)

            # selecting the two closest parking spots
            self.select_request.parking_spots = spots_with_distance[:2]
            # Call the select service
            self.future = self.select_client.call_async(self.select_request)
            rclpy.spin_until_future_complete(self, self.future)
            self.selected_spot_id = self.future.result().spot_id
            # Find the selected spot coordinates as per the selected spot id
            for spot in self.parking_spots.available_parking_spots:
                if spot.id == self.selected_spot_id: # checking if spot ID matches selected spot id 
                    self.last_point = (spot.x, spot.y)
            self.book_nearest_parking_spot(self.selected_spot_id) # booking the nearest parking spot

    def book_nearest_parking_spot(self, selected_spot_id):
        '''
        Send a booking request for the nearest parking spot and handle the response.
        '''
        self.get_logger().info("booking parking_spot")
        self.book_request.spot_id = selected_spot_id
        # Call the asynchronous service to book the parking spot
        self.future = self.book_client.call_async(self.book_request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result().booked

        if response is True: # check if the spot successfully booked
            self.get_logger().info("spot booked")
            self.book_timer.cancel()
            self.publish_path()
            self.publish_final_parking_point()
        else:
            self.get_logger().info("spot not booked")

    def publish_path(self):
        '''Publisher for the planned path.'''
        path_msg = Path()
        if self.selected_spot_id in self.availabile_path_dict:
            for node_id in self.availabile_path_dict[self.selected_spot_id]:
                node = self.nodes[node_id]
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = node.x
                pose_stamped.pose.position.y = node.y
                path_msg.poses.append(pose_stamped)
            self.get_logger().info(f"Publishing path: {path_msg}")
            self.path_pub.publish(path_msg)

    def publish_final_parking_point(self):
        '''Publish the final route point.'''
        self.route_data = SelectedParking()
        if self.last_point:
            point = Point()
            point.x = self.last_point[0]
            point.y = self.last_point[1]
            self.route_data.points.append(point)
            self.get_logger().info(f"Publishing final route point: {self.route_data}")
        else:
            self.get_logger().info("No final route point to publish")
        
        self.selected_parking_publisher.publish(self.route_data)

def main(args=None):
    '''
    Initialize ROS2, start the PathPlan node, and spin it using a MultiThreadedExecutor.
    '''
    rclpy.init(args=args)
    node = PathPlan()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()