import math
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations as tf
from sf_msgs.msg import VehiclesPosition

class VisualizationNode(Node):

    def __init__(self):
        super().__init__('VisualizationNode')  # Initialize the node with name 'VisualizationNode'

        # Set up QoS (Quality of Service) profile
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        # Create a publisher for MarkerArray messages on the '/cars_pos_viz' topic
        self.marker_array_pub = self.create_publisher(MarkerArray, '/cars_pos_viz', qos_profile)
        # Subscription to the '/all_vehicles_position' topic with a callback to 'cam_position'
        self.cam_subscription = self.create_subscription(VehiclesPosition, '/all_vehicles_position',
                                                         self.cam_position, qos_profile)
        # Store the current time stamp
        self.currentTimeStamp = self.get_clock().now().to_msg()
        # Initialize an empty MarkerArray
        self.marker_array = MarkerArray()

        self.get_logger().info("The Vehicle Marker Array is Being Published.....")

    def create_car_marker(self, vehicle_id, namespace, color, x, y, yaw_angle):
        car_marker = Marker()
        car_marker.type = 10  # Marker type (mesh resource)
        car_marker.action = 0  # Action (add/modify)
        car_marker.header.frame_id = 'map'  # Frame ID (coordinate frame)

        # Set the scale of the marker (dimensions)
        car_marker.scale.x = 0.001
        car_marker.scale.y = 0.001
        car_marker.scale.z = 0.001

        # Set the color and transparency of the marker
        car_marker.color.a = 1.0
        car_marker.mesh_resource = "file:/home/af/ros2_ws/src/sf_master/src/sf_model_city_map/sf_model_city_map/car_lowres.stl"
        car_marker.id = vehicle_id  # Marker ID
        car_marker.ns = namespace  # Namespace
        car_marker.color.r, car_marker.color.g, car_marker.color.b = color  # RGB color values

        # Set the position of the marker
        car_marker.pose.position.x = x
        car_marker.pose.position.y = y
        car_marker.pose.position.z = 0.45  # Z position (height)

        # Calculate quaternion from Euler angles for orientation
        quat = tf.quaternion_from_euler(math.pi / 2, 0, yaw_angle - (math.pi / 2))
        car_marker.pose.orientation.x = quat[0]
        car_marker.pose.orientation.y = quat[1]
        car_marker.pose.orientation.z = quat[2]
        car_marker.pose.orientation.w = quat[3]
        return car_marker

    def cam_position(self, msg):
        markers = []
        # Check vehicle ID and create markers accordingly
        if msg.vehicle_id == 9:
            markers.append(self.create_car_marker(
                vehicle_id=9,
                namespace="adapt_icon",
                color=(1.0, 1.0, 1.0),  # White color
                x=msg.x_coordinate,
                y=msg.y_coordinate,
                yaw_angle=msg.yaw_angle
            ))
        elif msg.vehicle_id == 10:
            markers.append(self.create_car_marker(
                vehicle_id=10,
                namespace="parkonomous_icon",
                color=(0.0, 0.0, 1.0),  # Blue color
                x=msg.x_coordinate,
                y=msg.y_coordinate,
                yaw_angle=msg.yaw_angle
            ))
        # Update the marker array and publish it
        self.marker_array.markers = markers
        self.marker_array_pub.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()