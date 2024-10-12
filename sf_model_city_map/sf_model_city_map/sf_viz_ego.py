import math
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker
import tf_transformations as tf
from mocap_msgs.msg import RigidBodies

class ModelCarsVizNode(Node):
    """
    Define a Node class for visualizing model cars
    """
    def __init__(self):
        """
        Initialize the node with a name 'minimal_subscriber
        """
        # Define initial values for attributes
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.z_orien = 0.0
        self.quat = [0.0, 0.0, 0.0, 1.0]  # Default quaternion [x, y, z, w]
        super().__init__('minimal_subscriber')
        # Define the QoS profile for the subscription and publisher
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        # Create a subscription to the '/pose_modelcars' topic with the given QoS profile
        self.subscription = self.create_subscription(RigidBodies, '/pose_modelcars',
                                                     self.position_orientation, qos_profile)
        # Create a publisher to the '/ego_pos_viz' topic with the given QoS profile
        self.publisher = self.create_publisher(Marker, '/ego_pos_viz', qos_profile)
        self.currentTimeStamp = self.get_clock().now().to_msg()
        # Create a marker for the car
        self.car_marker = self.create_marker()

        self.get_logger().info("The Ego-Vehicle Marker is Being Published.....")

    def create_marker(self):
        """
        Create a marker message for the car
        """
        car_marker = Marker()
        car_marker.ns = "car_icon"  # Namespace of the marker
        car_marker.id = 30  # Unique identifier for the marker
        car_marker.type = 10  # Type of marker (10 represents a mesh resource)
        car_marker.action = 0  
        car_marker.header.frame_id = 'map'  # Coordinate frame
        car_marker.scale.x = 0.001  # Scale in x direction
        car_marker.scale.y = 0.001  # Scale in y direction
        car_marker.scale.z = 0.001  # Scale in z direction
        car_marker.color.a = 1.0  # Alpha (transparency)
        car_marker.color.r = 1.0  # Red color component
        car_marker.color.g = 1.0  # Green color component
        car_marker.color.b = 0.0  # Blue color component
        car_marker.mesh_resource = "file:/home/af/ros2_ws/src/sf_master/src/sf_model_city_map/sf_model_city_map/car_lowres.stl"
        return car_marker

    def position_orientation(self, msg):
        """
        Callback function to process the RigidBodies message
        """
        # Iterate through the list of rigid bodies
        for car in msg.rigidbodies:
            # Check if the rigid body name is "7"
            if car.rigid_body_name == "7":
                # Extract and round the position coordinates
                self.x = round(car.pose.position.x, 2)
                self.y = round(car.pose.position.y, 2)
                self.z = round(car.pose.position.z, 2)
                # Convert the orientation from quaternion to Euler angles
                euler = tf.euler_from_quaternion([
                    car.pose.orientation.x,
                    car.pose.orientation.y,
                    car.pose.orientation.z,
                    car.pose.orientation.w
                ])
                # Extract the yaw angle (z orientation)
                self.z_orien = euler[2]
                # Update the marker's position and orientation
                self.position_update()

    def position_update(self):
        """
        Function to update the marker's position and orientation
        """
        self.car_marker.header.stamp = self.currentTimeStamp  # Update the timestamp
        self.car_marker.pose.position.x = round(self.x, 3)  # Update x position
        self.car_marker.pose.position.y = round(self.y, 3)  # Update y position
        self.car_marker.pose.position.z = 0.45  # Set a fixed z position
        # Convert Euler angles to quaternion
        self.quat = tf.quaternion_from_euler(math.pi/2, 0, self.z_orien - (math.pi/2))
        # Update the marker's orientation with the new quaternion
        self.car_marker.pose.orientation.x = self.quat[0]
        self.car_marker.pose.orientation.y = self.quat[1]
        self.car_marker.pose.orientation.z = self.quat[2]
        self.car_marker.pose.orientation.w = self.quat[3]
        self.publisher.publish(self.car_marker) # Publish the updated marker
        # print("Car_Marker_X: ", self.car_marker.pose.position.x)
        # print("Car_Marker_Y: ", self.car_marker.pose.position.y)
def main(args=None):
    """
    Main function to run the node
    """
    rclpy.init(args=args)  # Initialize the rclpy library
    node = ModelCarsVizNode()  # Create an instance of the node
    rclpy.spin(node)  # Spin the node to keep it running
    rclpy.shutdown()  # Shutdown the rclpy library

if __name__ == '__main__':
    main()