import rclpy
import unittest
from sf_model_city_map.sf_viz_ego import ModelCarsVizNode
from visualization_msgs.msg import Marker
from mocap_msgs.msg import RigidBodies, RigidBody



class Testmcmap(unittest.TestCase):
    
    def setUp(self):
        rclpy.init()
        self.ego_class = ModelCarsVizNode()

    def tearDown(self):
        self.ego_class.destroy_node()
        rclpy.shutdown()
        
    def test_position_orientation(self):
        # Mock message
        
        msg = RigidBodies()
        car = RigidBody()
        car.rigid_body_name = "7"
        car.pose.position.x = 1.0
        car.pose.position.y = 2.0
        car.pose.position.z = 3.0
        car.pose.orientation.x = 0.0
        car.pose.orientation.y = 0.0
        car.pose.orientation.z = 0.0
        car.pose.orientation.w = 1.0
        msg.rigidbodies.append(car)
        self.ego_class.quat = [0.0, 0.0, 0.0, 1.0]
        self.ego_class.position_update()
        #time.sleep(1)  # Wait for the marker to be published
        # Assert that the marker is published
        self.assertTrue(len(self.ego_class.car_marker.id) > 0)
    
        #self.assertEqual(self.ego_class.car_marker.id, 30)
        self.assertAlmostEqual(self.ego_class.car_marker.pose.position.x, 1.0)
        self.assertAlmostEqual(self.ego_class.car_marker.pose.position.y, 2.0)
        self.assertAlmostEqual(self.ego_class.car_marker.pose.position.z, 0.45)
        
        assert self.ego_class.car_marker.pose.orientation.x == 0.0 
        assert self.ego_class.car_marker.pose.orientation.y == 0.0
        assert self.ego_class.car_marker.pose.orientation.z == 0.0
        
        # Add more assertions to check other properties of the marker if needed

if __name__ == '__main__':
    unittest.main()