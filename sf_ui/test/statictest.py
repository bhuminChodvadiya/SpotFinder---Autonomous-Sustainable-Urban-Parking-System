import rclpy
from rclpy.node import Node
import PySimpleGUI as psg
from PIL import Image, ImageDraw, ImageFont
from sf_msgs.srv import SelectSpot

class UserInterface(Node):
    
    def __init__(self):
        super().__init__("ui")

        """
        Create a ROS service called "select_parking_spot" with the service type SelectSpot.
        """
        self.create_service(SelectSpot, 'select_parking_spot', self.select_parking_spot)
        """
        Timeout duration for the GUI in seconds
        """
        self.timeout = 300
        """
        Log that the user interface node is running
        """
        self.get_logger().info("user interface is running")

    def init_ui(self, parking_spots):
        
        r_spot = []
        """
        open image
        """
        im = Image.open("//home/af/ros2_ws/src/sf_master/src/sf_ui/resource/mc_map.png")
        draw = ImageDraw.Draw(im)
        font = ImageFont.truetype(r"/home/af/ros2_ws/src/sf_master/src/sf_ui/resource/OpenSans-Medium.ttf", 30)

        for parking_spot in parking_spots:
            """
            draw bounding box form top left to bottom right of coordinates of parking_Spot
            """
            r = psg.Radio(f"Spot {parking_spot.id+1}: {round(parking_spot.distance,2)}m away from current location", "spot", key=parking_spot.id)
            r_spot.append(r)
            """
            highlight spot in map
            """
            if parking_spot.id == 0:
                draw.rectangle(((502, 407), (542, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 15,int(parking_spot.x*100) - 15], str(parking_spot.id+1), font=font, fill="green")
            if parking_spot.id == 1:
                draw.rectangle(((548, 407), (590, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 15,int(parking_spot.x*100) - 15], str(parking_spot.id+1), font=font, fill="green")
            if parking_spot.id == 2:
                draw.rectangle(((594, 407), (635, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 15,int(parking_spot.x*100) - 15], str(parking_spot.id+1), font=font, fill="green")
            if parking_spot.id == 3:
                draw.rectangle(((640, 407), (680, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 23,int(parking_spot.x*100) - 15], str(parking_spot.id+1), font=font, fill="green")
                
            print(parking_spot.y)    

        """
        save image
        """
        im.save("//home/af/ros2_ws/src/sf_master/src/sf_ui/resource/mc_map_bbox.png")

        """
        Initialize the user interface layout based on available parking spots
        """
        l1 =  psg.Text("Model City Map", font=("Helvetica", 14))
        im1 = psg.Image("//home/af/ros2_ws/src/sf_master/src/sf_ui/resource/mc_map_bbox.png", key="map")
        b1 = psg.Button("confirm")
    
        """
        Define the layout of the user interface
        """
        layout = [[l1],[im1], [[radio] for radio in r_spot], [b1]]
        return layout

    def select_parking_spot(self, request, response):

        """
        Method to handle selection of a parking spot & Initialize spot ID with the first parking spot ID from the request
        """
        self.get_logger().info("selection of parking spot")
        spot_id = request.parking_spots[0].id
        window = psg.Window("SpotFinder", self.init_ui(request.parking_spots))

        while True:
            event, values = window.read(timeout=1000*self.timeout)
            if event == psg.WIN_CLOSED or event == 'Cancel':
                break
            if event in ('__TIMEOUT__',):
                window.close()
                break
            """
            Get the ID of the selected parking spot
            """
            spot_id = list(values.keys())[list(values.values()).index(True)]
            window.close()
        response.spot_id = spot_id
        self.get_logger().info("selected spot with id : " + str(response.spot_id))
        return response
       
    def main(args=None):
        rclpy.init(args=args)
        node = UserInterface()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()