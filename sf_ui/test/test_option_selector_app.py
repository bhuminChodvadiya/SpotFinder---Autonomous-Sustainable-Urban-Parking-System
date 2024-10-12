import sys
import unittest
from unittest.mock import MagicMock, patch

# Add the parent directory of 'sf_ui' to the Python path
sys.path.append('/home/khan/tm_ws/src/sf_master/src/sf_ui/sf_ui')

from sf_ui import UserInterface  

class TestUserInterface(unittest.TestCase):
    @patch('sf_ui.psg')
    def test_init_ui(self, mock_psg):
        # Mock parking spots data
        parking_spots = [
            MagicMock(id=0, distance=10),
            MagicMock(id=1, distance=20),
            MagicMock(id=2, distance=30)
        ]

        # Create a UserInterface instance
        node = UserInterface()

        # Call init_ui with mock data
        layout = node.init_ui(parking_spots)

        # Check if PySimpleGUI components are called correctly
        mock_psg.Text.assert_called_once_with("Available parking Spots")
        mock_psg.Button.assert_called_once_with("confirm")
        mock_psg.Radio.assert_any_call("Spot 1: 10m away from current location", "spot", key=0, default=True)
        mock_psg.Radio.assert_any_call("Spot 2: 20m away from current location", "spot", key=1, default=False)
        mock_psg.Radio.assert_any_call("Spot 3: 30m away from current location", "spot", key=2, default=False)

        # Check if layout is generated correctly
        expected_layout = [
            [mock_psg.Text.return_value],
            [[mock_psg.Radio.return_value] * 3],
            [mock_psg.Button.return_value]
        ]
        self.assertEqual(layout, expected_layout)

if __name__ == '__main__':
    unittest.main()
