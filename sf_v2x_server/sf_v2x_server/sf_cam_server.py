"""This Module is simulating the CAM nessages of other """
import rclpy
import pymap3d as pm
from rclpy.node import Node

from mocap_msgs.msg import RigidBodies
from v2x.msg import CAM
import tf_transformations as tf

class CAMGeneratorServerNode(Node):
    """
    This is node simulating other traffic participants by publishign cam messages
    """
    def __init__(self):
        super().__init__("cam_server")

        # subscriber
        self.subscription = self.create_subscription(
            RigidBodies, '/pose_modelcars', self.vehicle_position_callback, 10)

        # publisher
        self.cam_publisher = self.create_publisher(CAM, "/common_topic_name", 10)

        # init variables
        # reference positition for converting local to WGS-84 coordinate system
        self.lat_0 = 50.24132213367954
        self.lon_0 = 11.321265180951718

        self.get_logger().info("CAM Server node is running...")

    def vehicle_position_callback(self, rigid_body_msg):
        """
        this function is converting the rigid_body message to 
        multiple cam messages and publishing those messages
        """

        # for each element in rigidbodies publish a CAM-Message
        for rigid_body in rigid_body_msg.rigidbodies:
            v_id = rigid_body.rigid_body_name
            v_pos_x = rigid_body.pose.position.x
            v_pos_y = rigid_body.pose.position.y


            # Convert local coordinates to WGS-84 coordinates
            global_position = pm.enu2geodetic(e= v_pos_x, n=v_pos_y, u=0, lat0= self.lat_0, lon0=self.lon_0, h0=0)

            v_latitude = int(global_position[0] * (10 **7))
            v_longitude = int(global_position[1] * (10 **7))

            q_x = rigid_body.pose.orientation.x
            q_y = rigid_body.pose.orientation.y
            q_z = rigid_body.pose.orientation.z
            q_w = rigid_body.pose.orientation.w

            yaw_angle =  tf.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
            if yaw_angle < 0:
                yaw_angle += 360

            cam_msg = CAM()
            # Vehicle id of the behicle transmitting CAM
            cam_msg.header.station_id = int(v_id)
            # Global position of vehicle in longitude
            cam_msg.cam.cam_parameters.basic_container.reference_position.longitude = v_longitude
            # Global position of vehicle in latitude
            cam_msg.cam.cam_parameters.basic_container.reference_position.latitude = v_latitude

            # heading of the vehicle
            cam_msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value = int(yaw_angle * 10)

            # Publish the CAM message
            self.cam_publisher.publish(cam_msg)

def main(args=None):
    """
    this function is starting the node
    """
    rclpy.init(args=args)
    node = CAMGeneratorServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
