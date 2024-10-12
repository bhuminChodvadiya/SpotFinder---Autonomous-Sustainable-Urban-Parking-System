import rclpy
from rclpy.node import Node
from sf_msgs.msg import StreetInfoContainer, StreetInfo, VehiclesPosition    # Import messages

class TrafficMonitor(Node):

    def __init__(self):
        super().__init__('sf_traffic_monitor')

        # init subscriber
        self.subscription = self.create_subscription(VehiclesPosition, "/all_vehicles_position", self.vehicles_position_callback, 1)  # Subscribing to VehiclesPosition topic

        # init publisher
        self.publisher = self.create_publisher(StreetInfoContainer, "/traffic_density", 10)  # Create publisher for StreetInfoContainer topic
        
        # create timer
        timer_period = 0.1
        self.create_timer(timer_period, self.calculate_density)

        # init variables
        self.vehicles_positions = []

        self.name_dict = {  # Map street node names
            0: "JI",
            1: "IH",
            2: "HG",
            3: "JC",
            4: "ID",
            5: "HE",
            6: "GF",
            7: "CD",
            8: "DE",
            9: "EF",
            10: "CA",
            11: "DB",
            12: "AB",
        }

        self.area_dict = {  # Number of blocks(Size is 1*1(m)) considered in between different street segments.
            0: 1,
            1: 1,
            2: 2,
            3: 3.5,
            4: 3.5,
            5: 4,
            6: 4,
            7: 1,
            8: 1,
            9: 2,
            10: 4.5,
            11: 4.5,
            12: 1,
        }

        self.get_logger().info("The traffic node is running.....")   # Log message

    def calculate_density(self):
        traffic_density_container = StreetInfoContainer()
        traffic_densities = []

        for i in range(13):      
            traffic_density = StreetInfo()
            traffic_density.street_name = self.name_dict.get(i)
            traffic_density.area = float(self.area_dict.get(i))
            traffic_density.number_of_vehicles = 0  # Initialize number of vehicles
            traffic_densities.append(traffic_density)

        for vehicle_position in self.vehicles_positions:
            x_pos = vehicle_position.get("x_coordinate")
            y_pos = vehicle_position.get("y_coordinate")

            # Define a list of conditions and corresponding actions
            conditions_actions = [
                ((0 < x_pos < 1) and (1 < y_pos < 2), 0),  # Node J & I
                ((0 < x_pos < 1) and (3 < y_pos < 4), 1),  # Node I & H
                ((0 < x_pos < 1) and (5 < y_pos < 7), 2),  # Node H & G
                ((0 < x_pos < 3.5) and (0 < y_pos < 1), 3), # Node J & C
                ((0 < x_pos < 3.5) and (2 < y_pos < 3), 4), # Node I & D
                ((0 < x_pos < 4) and (4 < y_pos < 5), 5),   # Node H & E
                ((0 < x_pos < 4) and (7 < y_pos < 8), 6),   # Node G & F
                ((3 < x_pos < 4) and (1 < y_pos < 2), 7),   # Node C & D
                ((3 < x_pos < 4) and (3 < y_pos < 4), 8),   # Node D & E
                ((3 < x_pos < 4) and (5 < y_pos < 7), 9),   # Node E & F
                ((3.5 < x_pos < 7) and (0 < y_pos < 1), 10),# Node C & A
                ((3.5 < x_pos < 7) and (2 < y_pos < 3), 11),# Node D & B
                ((6 < x_pos < 7) and (1 < y_pos < 2), 12)   # Node A & B
            ]

            # Iterate over the list and update traffic densities based on conditions
            for condition, index in conditions_actions:
                if condition:
                    traffic_densities[index].number_of_vehicles += 1

        for traffic_density in traffic_densities:
            traffic_density.density = traffic_density.number_of_vehicles / traffic_density.area

        traffic_density_container.traffic_densities = traffic_densities
        self.publisher.publish(traffic_density_container)

    def vehicles_position_callback(self, vehicles_position_msg:VehiclesPosition):
        vehicle_id = vehicles_position_msg.vehicle_id
        for vehicle_position in self.vehicles_positions:
            # if vehicle is in vehicles_positions list then update position
            if vehicle_position.get("id") == vehicle_id:
                vehicle_position["x_coordinate"] = vehicles_position_msg.x_coordinate
                vehicle_position["y_coordinate"] = vehicles_position_msg.y_coordinate
                return

        # vehicle is not in vehicles_positions
        self.vehicles_positions.append({
            "id":vehicle_id,
            "x_coordinate":vehicles_position_msg.x_coordinate,
            "y_coordinate":vehicles_position_msg.y_coordinate 
        })

def main(args=None):    
    rclpy.init(args=args)   
    node = TrafficMonitor()   
    rclpy.spin(node)   
    rclpy.shutdown()    


if __name__ == '__main__':
    main()
