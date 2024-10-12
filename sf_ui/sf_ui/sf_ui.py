import rclpy
from rclpy.node import Node
import PySimpleGUI as psg
from sf_msgs.srv import SelectSpot
from PIL import Image, ImageDraw, ImageFont

class UserInterface(Node):
    def __init__(self):
        super().__init__("ui")
        # Create a ROS service called "select_parking_spot" with the service type SelectSpot.
        self.create_service(SelectSpot, 'select_parking_spot', self.select_parking_spot)
        # self.create_subscription(EgoPosition, '/ego_position', self.modelcar_visualization_callback, 10)
        # self.create_subscription(Path, "/planned_path", self.path_visualization_callback, 10)
        # Timeout duration for the GUI in seconds
        self.timeout = 300
        # Log that the user interface node is running
        self.get_logger().info("user interface is running")
        # self.modelcar_x = 0
        # self.modelcar_y = 0
        # self.modelcar_yaw = 0
        # self.path_waypoints = []

    # def modelcar_visualization_callback(self, egoposition_msg):
    #     self.modelcar_x = egoposition_msg.x_coordinate * 100
    #     self.modelcar_y = egoposition_msg.y_coordinate * 100
    #     self.modelcar_yaw = egoposition_msg.yaw_angle

    # def path_visualization_callback(self, path_data):
    #     self.path_waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path_data.poses]
    #     print(self.path_waypoints)


    def parking_visualization(self, parking_spots):
        r_spot = []
        # open image
        im = Image.open("//home/af/ros2_ws/src/sf_master/src/sf_ui/resource/mc_map.png")
        draw = ImageDraw.Draw(im)
        font = ImageFont.truetype(r"/home/af/ros2_ws/src/sf_master/src/sf_ui/resource/OpenSans-Medium.ttf", 30)

        for parking_spot in parking_spots:
            # draw bounding box form top left to bottom right of coordinates of parking_Spot
            r = psg.Radio(f"Spot {parking_spot.id+1}: {round(parking_spot.distance,2)}m away from current location",
                          "spot", key=parking_spot.id)
            r_spot.append(r)
            # highlight spot in map
            if parking_spot.id == 0:
                draw.rectangle(((502, 407), (542, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 15,int(parking_spot.x*100) - 15],
                          str(parking_spot.id+1), font=font, fill="green")
            if parking_spot.id == 1:
                draw.rectangle(((548, 407), (590, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 15,int(parking_spot.x*100) - 15],
                          str(parking_spot.id+1), font=font, fill="green")
            if parking_spot.id == 2:
                draw.rectangle(((594, 407), (635, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 15,int(parking_spot.x*100) - 15],
                          str(parking_spot.id+1), font=font, fill="green")
            if parking_spot.id == 3:
                draw.rectangle(((640, 407), (680, 495)), fill="lightgreen", outline="green")
                draw.text([int(parking_spot.y*100) - 23,int(parking_spot.x*100) - 15],
                          str(parking_spot.id+1), font=font, fill="green")

            print(parking_spot.y)

        # save image
        im.save("//home/af/ros2_ws/src/sf_master/src/sf_ui/resource/mc_map_bbox.png")
        # Initialize the user interface layout based on available parking spots
        l1 =  psg.Text("Model City Map", font=("Helvetica", 14))
        im1 = psg.Image("//home/af/ros2_ws/src/sf_master/src/sf_ui/resource/mc_map_bbox.png",
                        key="map")
        b1 = psg.Button("confirm")
        # Define the layout of the user interface
        layout = [[l1],[im1], [[radio] for radio in r_spot], [b1]]

        return layout

    def select_parking_spot(self, request, response):
        # Method to handle selection of a parking spot
        self.get_logger().info("selection of parking spot")

        # Initialize spot ID with the first parking spot ID from the request
        spot_id = request.parking_spots[0].id

        window = psg.Window("SpotFinder", self.parking_visualization(request.parking_spots))

        while True:
            event, values = window.read(timeout=1000*self.timeout)

            if event == psg.WIN_CLOSED or event == 'Cancel':
                break

            if event in ('__TIMEOUT__',):
                window.close()
                break

            # Get the ID of the selected parking spot
            spot_id = list(values.keys())[list(values.values()).index(True)]
            window.close()

        response.spot_id = spot_id
        self.get_logger().info("selected spot with id : " + str(response.spot_id))

        return response

# def draw_car(graph, x, y):
#     graph.draw_oval((x - 20, y - 10), (x + 20, y + 10), line_color='blue', fill_color='blue')

# def draw_path(graph, waypoints):
#     if waypoints:
#         for i in range(len(waypoints) - 1):
#             graph.draw_line(waypoints[i], waypoints[i + 1], color='red', width=2)

# def update_gui(window,node):
#     while True:
#         event, values = window.read(timeout=100)
#         if event == psg.WIN_CLOSED:
#             break

#         window["map"].erase()
#         window["map"].update("/home/af/ros2_ws/src/sf_master/src/sf_ui/resource/mc_map.png")
    

#         draw_car(window["map"], node.modelcar_x, node.modelcar_y)
#         draw_path(window["map"], node.path_waypoints)
#         rclpy.spin_once(node, timeout_sec=0.1)
#     window.close()

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



