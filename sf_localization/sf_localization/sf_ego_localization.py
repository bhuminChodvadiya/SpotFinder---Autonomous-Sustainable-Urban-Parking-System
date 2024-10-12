import rclpy
from rclpy.node import Node
import pymap3d as pm                            # pymap3d library to convert into global coordinate
import tf_transformations as tf                 # tranformation library for euler and quarternions
from mocap_msgs.msg import RigidBodies          # Mocap masg for subscriptions of vehicle position
from sf_msgs.msg import EgoPosition             # Sf msgs for publition of vehicle data

class Localization(Node):
    '''
    This Node subscribes data from Optitrack motion capture system and publishes the ego vehicle's global position.
    '''
    def __init__(self):
        super().__init__('sf_ego_localization')
        self.subscription = self.create_subscription(RigidBodies, '/pose_modelcars', self.rigid_bodies_callback, 1)
        self.ego_publisher = self.create_publisher(EgoPosition, "/ego_position", 10)

        self.get_logger().info("The Localization node is running.....")
        
        # Vehicle ID
        self.vehicle_id= "7"
        # Initial latitude, longitude  values (Respected to Model City's origin(0,0))
        self.lat_0 = 50.24132213367954
        self.lon_0 = 11.321265180951718
        self.h_0 = 0    # Initial height (sea level)
        
    def rigid_bodies_callback(self, rigid_bodies_msg:RigidBodies):
        """
        Process rigid body data from Optitrack system,
        convert the Model city's coordinates to global coordinates,
        and publish the ego vehicle's Pose (position and orientation)
        """
        rigid_body = None
        # Iterate through the list of rigid bodies to find the one with the matching vehicle ID.
        for rigid_body_temp in rigid_bodies_msg.rigidbodies:
            if rigid_body_temp.rigid_body_name == self.vehicle_id:
                rigid_body = rigid_body_temp
                break   # Exit the loop once the matching rigid body is found

        ego_position_msg = EgoPosition()
        # (x,y) position of vehicle from rigid body
        pos = (rigid_body.pose.position.x, rigid_body.pose.position.y)
        # Convert to global coordinates using pymap3d
        global_position = pm.enu2geodetic(e= pos[0], n=pos[1], u=0, lat0= self.lat_0, lon0=self.lon_0, h0=self.h_0)
        # Extract quaternion components
        q_x = rigid_body.pose.orientation.x
        q_y = rigid_body.pose.orientation.y
        q_z = rigid_body.pose.orientation.z
        q_w = rigid_body.pose.orientation.w
        # yaw angle according to model city co-ordinates
        #|-----------y----------
        #|           0
        #x  90             -90
        #|       180|-180
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        yaw = tf.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
        # Populate ego position message
        ego_position_msg.x_coordinate = pos[0]
        ego_position_msg.y_coordinate = pos[1]
        ego_position_msg.yaw_angle = yaw
        ego_position_msg.longitude = int(global_position[1] * (10 **7))
        ego_position_msg.latitude = int(global_position[0] * (10 **7))
        # Publish ego position message
        self.ego_publisher.publish(ego_position_msg)

def main(args=None):
    """
    Initialize , start and keep it spinning until node is shut down.
    """
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()