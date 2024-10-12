import unittest
import rclpy
from unittest.mock import MagicMock, patch
import PySimpleGUI as psg
from sf_ui.sf_ui import UserInterface  

class TestUserInterface(unittest.TestCase):

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()
    
    def test_init_ui(self):
        # Mock the parking_spots data
        parking_spots = [
            MagicMock(id=0, distance=10),
            MagicMock(id=1, distance=20),
            MagicMock(id=2, distance=30)
        ]
        
        # Create a UserInterface instance
        node = UserInterface()

        # Call the init_ui function with mock data
        layout = node.init_ui(parking_spots)

        # Verify the generated layout
        expected_layout = [
            [MagicMock(text="Available parking Spots")],
            [
                [MagicMock(key=0, default=True)],
                [MagicMock(key=1)],
                [MagicMock(key=2)]
            ],
            [MagicMock(text="confirm")]
        ]
        self.assertEqual(layout, expected_layout)

    def test_select_parking_spot(self):
        # Mocking request and response objects
        request = MagicMock()
        request.parking_spots = [MagicMock(id=0)]

        response = MagicMock()

        # Mocking PySimpleGUI components
        mock_window = MagicMock()
        mock_window.read.side_effect = [
            (None, None),  # Simulate window closing without any events
            ('__TIMEOUT__', None),   # Simulate timeout
            ({0: True}, None)        # Simulate user selecting a parking spot
        ]
        mock_psg = MagicMock()
        mock_psg.Window.return_value = mock_window

        # Create a UserInterface instance
        node = UserInterface()

        # Patch PySimpleGUI import within the sf_ui module
        with patch('sf_ui.sf_ui.psg', mock_psg):
            # Call the select_parking_spot function
            node.select_parking_spot(request, response)

        # Verify the response
        self.assertEqual(response.spot_id, 0)

        # Verify the window was closed after selection
        mock_window.close.assert_called_once()

        # Verify logging messages
        self.assertTrue(node.get_logger().info("selection of parking spot"))
        self.assertTrue(node.get_logger().info("selected spot with id : 0"))

if __name__ == '__main__':
    unittest.main()
