import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sf_msgs.msg import EgoPosition
from time import sleep

class MITestNode(Node):
    def __init__(self):
        super().__init__('integration_test_node')

        # Setting up publishers to simulate the inputs to MobileInterface
        self.position_publisher = self.create_publisher(EgoPosition, '/ego_position', 10)
        self.completion_publisher = self.create_publisher(Bool, '/parking_reached_msg', 10)
        
        # Delay to allow subscribers on the other nodes to be ready

        sleep(2)
        self.run_tests()

    def run_tests(self):
        self.test_position_update()
        self.test_parking_reached_msg()

    #IT_MI_001:test EgoPosition update handling
    def test_position_update(self):
        print("Testing position update...")
        position_msg = EgoPosition()
        position_msg.x = 10.0
        position_msg.y = 5.0
        position_msg.z = 0.0
        self.position_publisher.publish(position_msg)
        # Wait to see if the MobileInterface node handles the update
        sleep(1)
        print("Position update test published.")
        
    #IT_MI_002:test parking position reached message handling
    def test_parking_position_reached(self):
        print("Testing parking position reached...")
        parking_msg = Bool()
        parking_msg.data = True
        self.completion_publisher.publish(parking_msg)
        # Wait to see if the MobileInterface node reacts to the parking position reached message
        sleep(1)
        print("Parking position reached test published.")

def main(args=None):
    rclpy.init(args=args)
    integration_test_node = MITestNode()
    try:
        rclpy.spin(integration_test_node)
    except KeyboardInterrupt:
        print('Integration testing stopped manually.')
    finally:
        integration_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
