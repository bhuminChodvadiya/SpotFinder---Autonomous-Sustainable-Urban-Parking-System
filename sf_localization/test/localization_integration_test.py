import pytest
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mocap_msgs.msg import RigidBodies
from sf_msgs.msg import EgoPosition
from your_package.localization_node import Localization  # Replace with actual import path

class TestLocalizationIntegration:

    @classmethod
    def setup_class(cls):
        # Initialize ROS2 context and node
        rclpy.init()
        cls.node = Localization()

        # Create a MultiThreadedExecutor to run the node in a separate thread
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)

        # Start the node in a separate thread
        cls.node_thread = threading.Thread(target=cls.executor.spin)
        cls.node_thread.start()

        # Wait for node to be ready
        time.sleep(2)  # Adjust as needed for node initialization time

    @classmethod
    def teardown_class(cls):
        # Shutdown ROS2 node and context
        cls.node.get_logger().info("Shutting down test node...")
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

        # Wait for node thread to join
        cls.node_thread.join()

    def test_integration_valid_rigidbody(self):
        # Test case for valid rigid body data
        rigid_bodies_msg = RigidBodies()
        rigid_body = rigid_bodies_msg.rigidbodies.add()
        rigid_body.rigid_body_name = "7"
        rigid_body.pose.position.x = 1.0
        rigid_body.pose.position.y = 2.0
        rigid_body.pose.orientation.x = 0.0
        rigid_body.pose.orientation.y = 0.0
        rigid_body.pose.orientation.z = 0.0
        rigid_body.pose.orientation.w = 1.0

        # Publish the RigidBodies message
        publisher = self.node.create_publisher(RigidBodies, '/pose_modelcars', 10)
        publisher.publish(rigid_bodies_msg)

        # Wait for message processing
        time.sleep(2)  # Adjust as needed for message processing time

        # Assert that EgoPosition message was published
        ego_position_msg = self.node.ego_publisher.get_last_msg()
        assert ego_position_msg is not None
        assert ego_position_msg.x_coordinate == 1.0
        assert ego_position_msg.y_coordinate == 2.0

    def test_integration_missing_rigidbody(self):
        # Test case for missing rigid body data
        rigid_bodies_msg = RigidBodies()

        # Publish the RigidBodies message
        publisher = self.node.create_publisher(RigidBodies, '/pose_modelcars', 10)
        publisher.publish(rigid_bodies_msg)

        # Wait for message processing
        time.sleep(2)  # Adjust as needed for message processing time

        # Assert that no EgoPosition message was published
        ego_position_msg = self.node.ego_publisher.get_last_msg()
        assert ego_position_msg is None

    def test_integration_continuous_updates(self):
        # Test case for continuous localization updates
        for i in range(5):
            rigid_bodies_msg = RigidBodies()
            rigid_body = rigid_bodies_msg.rigidbodies.add()
            rigid_body.rigid_body_name = "7"
            rigid_body.pose.position.x = float(i)
            rigid_body.pose.position.y = float(i)
            rigid_body.pose.orientation.x = 0.0
            rigid_body.pose.orientation.y = 0.0
            rigid_body.pose.orientation.z = 0.0
            rigid_body.pose.orientation.w = 1.0

            # Publish the RigidBodies message
            publisher = self.node.create_publisher(RigidBodies, '/pose_modelcars', 10)
            publisher.publish(rigid_bodies_msg)

            # Wait for message processing
            time.sleep(1)  # Adjust as needed for message processing time

            # Assert that EgoPosition message was published
            ego_position_msg = self.node.ego_publisher.get_last_msg()
            assert ego_position_msg is not None
            assert ego_position_msg.x_coordinate == float(i)
            assert ego_position_msg.y_coordinate == float(i)

    def test_integration_non_matching_rigid_body_id(self):
        # Test case for non-matching rigid body ID
        rigid_bodies_msg = RigidBodies()
        rigid_body = rigid_bodies_msg.rigidbodies.add()
        rigid_body.rigid_body_name = "10"  # Non-matching ID
        rigid_body.pose.position.x = 1.0
        rigid_body.pose.position.y = 2.0
        rigid_body.pose.orientation.x = 0.0
        rigid_body.pose.orientation.y = 0.0
        rigid_body.pose.orientation.z = 0.0
        rigid_body.pose.orientation.w = 1.0

        # Publish the RigidBodies message
        publisher = self.node.create_publisher(RigidBodies, '/pose_modelcars', 10)
        publisher.publish(rigid_bodies_msg)

        # Wait for message processing
        time.sleep(2)  # Adjust as needed for message processing time

        # Assert that no EgoPosition message was published
        ego_position_msg = self.node.ego_publisher.get_last_msg()
        assert ego_position_msg is None

# If this script is run directly, execute the tests
if __name__ == '__main__':
    pytest.main()
