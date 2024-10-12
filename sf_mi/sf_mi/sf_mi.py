import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sf_msgs.msg import EgoPosition
from flask import Flask, request, jsonify, abort
from threading import Thread

class MobileInterface(Node):
    def __init__(self):
        super().__init__('mobile_interface')
        
        # Flask application initialization
        self.app = Flask(__name__)
        self.define_routes()

        #Subscribers
        self.ego_position = None
        self.parking_position_reached = False

        #Variables
        self.create_subscription(EgoPosition, '/ego_position', self.position_callback, 10)
        self.create_subscription(Bool, '/parking_position_reached', self.parking_callback, 10)

    def define_routes(self):
        #Defines the Flask routes for HTTP requests, handling user authentication,registration, and vehicle status retrieval.
        @self.app.route('/get_vehicle_status', methods=['GET'])
        def get_vehicle_status():
            if self.ego_position is None:
                abort(500, description="Position of the vehicle is not known")
            response = {
                "x_coordinate": self.ego_position.x,
                "y_coordinate": self.ego_position.y,
                "parking_position_reached": self.parking_position_reached
            }
            return jsonify(response)

        @self.app.route('/login', methods=['POST'])
        def login():
            #Authenticates a user based on email and password provided.
            data = request.get_json()
            email = data['email']
            password = data['password']
            result = self.authenticate_user(email, password)
            if result == "Login successful.":
                return jsonify({'status': 'ok', 'message': result})
            else:
                return abort(401, description=result)

        @self.app.route('/sign_in', methods=['POST'])
        def sign_in():
            #Registers a new user if the email is not already in use.
            data = request.get_json()
            email = data['email']
            password = data['password']
            result = self.register_user(email, password)
            if result == "Registration successful.":
                return jsonify({'status': 'ok', 'message': result})
            else:
                return abort(409, description=result)

    def position_callback(self, ego_position_msg):
        #Callback to update the vehicle's position based on ROS topic updates.
        self.ego_position = ego_position_msg

    def parking_callback(self, parking_position_reached_msg):
        #Callback to update the vehicle's parking status based on ROS topic updates.
        self.parking_position_reached = parking_position_reached_msg.data

    def register_user(self, email, password, file_path='src/sf_mi/resource/User_data'):
        #Registers a user by writing their email and password to a file.
        try:
            with open(file_path, 'a+') as file:
                file.seek(0)  
                users = file.readlines()
                if any(email == line.strip().split(':')[0] for line in users):
                    return "Email already registered."
                
                file.write(f"{email}:{password}\n")  
                return "Registration successful."
        except IOError as e:
            return f"An error occurred: {str(e)}"

    def authenticate_user(self, email, password, file_path='src/sf_mi/resource/User_data'):
        #Authenticates a user by verifying their email and password against a file.
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    saved_email, saved_password = line.strip().split(':')
                    if email == saved_email and password == saved_password:
                        return "Login successful."
                return "Invalid email or password."
        except IOError as e:
            return f"An error occurred: {str(e)}"

    def run_flask(self):
        #Starts the Flask web server.
        self.app.run(host='0.0.0.0', port=8080, debug=False)

if __name__ == '__main__':
    rclpy.init()
    mobile_interface = MobileInterface()
    flask_thread = Thread(target=mobile_interface.run_flask)
    flask_thread.start()
    rclpy.spin(mobile_interface)
    flask_thread.join()

   
    
    
