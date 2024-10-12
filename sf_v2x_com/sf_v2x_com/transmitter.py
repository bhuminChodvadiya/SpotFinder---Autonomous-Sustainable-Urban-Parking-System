import rclpy
from rclpy.node import Node

# messages
from sf_msgs.msg import EgoPosition
from v2x.msg import CAM, CPM, PerceivedObject, ObjectClassWithConfidence
from vision_msgs.msg import Detection2DArray

class Tranmitter(Node):
    def __init__(self):
        super().__init__("transmitter")
        # Initialize attributes
        self.lon = 0.0
        self.lat = 0.0
        self.yaw = 0.0
        # subscriber
        self.create_subscription(EgoPosition, "/ego_position", self.ego_position_callback, 10)
        self.create_subscription(Detection2DArray, "/detectnet/detections",
                                 self.detectnet_callback, 10)
        # publisher
        self.cam_publisher = self.create_publisher(CAM, "/cam_msgs", 10)
        self.cpm_publisher = self.create_publisher(CPM, "/cpm_msgs", 10)

        self.get_logger().info("Transmitter node is running...")

    # Detectnet id to CPM message id relation
    # detectnect trained classes that can be transmitted as CPM are as follows:
        # 0 - background
        # 1 - car
        # 4 - person
        # 5 - traffic_lights

    # CPM classes are described as follows:
    object_class_dict = {
        0 : 0, #unknown       - information about traffic participant is not provided,
		4 : 1, #pedestrian    - human being not using a mechanical device for their trip (VRU profile 1),
		1 : 5, #passengerCar  - small passenger vehicles as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class M1,
		5 : 15 #roadSideUnit  - infrastructure typically positioned outside of the drivable roadway (e.g. on a gantry, on a pole, on a stationary road works trailer); the infrastructure is static during the entire operation period of the ITS-S (e.g. no stop and go activity)
    }

    # publish CAM Message
    def ego_position_callback(self, ego_position_msg:EgoPosition):
        # Create a CAM message
        cam_msg = CAM()

        # Reference latitude and longitude
        self.lon = ego_position_msg.longitude
        self.lat = ego_position_msg.latitude

        # Convert the yaw angle from model city co-ordinate to match the ETSI heading value
        # As per https://forge.etsi.org/rep/ITS/asn1/cpm_ts103324/-/blob/v2.1.1/docs/ETSI-ITS-CDD.md#HeadingValue
        self.yaw = ego_position_msg.yaw_angle * 10
        if 0.0 < self.yaw <= -90.0:
            self.yaw = abs(self.yaw) + 270
        elif -90.0 < self.yaw <=-180.0:
            self.yaw = abs(self.yaw + 90)
        else:
            self.yaw = 270 - self.yaw

        # Ego_vehicle id = 7
        cam_msg.header.station_id = 7

        # Global position of vehicle in longitude
        cam_msg.cam.cam_parameters.basic_container.reference_position.longitude = self.lon
        # Global position of vehicle in latitude
        cam_msg.cam.cam_parameters.basic_container.reference_position.latitude = self.lat
        # Heading value of the vehicle
        cam_msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value = int(self.yaw)
        # Publish the CAM message
        self.cam_publisher.publish(cam_msg)

    def detectnet_callback(self, msg):
        """
        publish CPM Message
        """
        cpm_msg = CPM()

        #message id is set to 14 as per ETSI TS 102 894-2 V2.1.1 - ITS Common Data Dictionary (CDD) for cpm
        cpm_msg.header.message_id = 14

        #reference position for the cpm message
        cpm_msg.payload.management_container.reference_position.longitude = self.lon
        cpm_msg.payload.management_container.reference_position.latitude = self.lat

        #message segment information
        cpm_msg.payload.management_container.segmentation_info.total_msg_no = 1
        cpm_msg.payload.management_container.segmentation_info.this_msg_no = 1

        cpm_msg.payload.cpm_containers.originating_vehicle_container_is_present = True

        #The unavailable values are set to their corresponding values as per ETSI TS 102 894-2 V2.1.1 - ITS Common Data Dictionary (CDD)
        #https://www.etsi.org/deliver/etsi_ts/102800_102899/10289402/02.01.01_60/ts_10289402v020101p.pdf
        #These message values are intended for future use, when the corresponding data is available

        #originating vehicle orientation
        cpm_msg.payload.cpm_containers.originating_vehicle_container.vehicle_orientation_angle.wgs84_angle_value = self.yaw #unavailable
        cpm_msg.payload.cpm_containers.originating_vehicle_container.vehicle_orientation_angle.angle_confidence = 127 #unavailable

        #originating vehicle pitch
        cpm_msg.payload.cpm_containers.originating_vehicle_container.pitch_angle.cartesian_angle_value = 3601 #unavailable
        cpm_msg.payload.cpm_containers.originating_vehicle_container.pitch_angle.angle_confidence = 127 #unavailable

        #originating vehicle roll
        cpm_msg.payload.cpm_containers.originating_vehicle_container.roll_angle.cartesian_angle_value = 3601 #unavailable
        cpm_msg.payload.cpm_containers.originating_vehicle_container.roll_angle.angle_confidence = 127 #unavailable

        cpm_msg.payload.cpm_containers.originating_rsu_container_is_present = False

        #sensor information
        cpm_msg.payload.cpm_containers.sensor_information_container.sensor_information_container.type = 3 #monovideo

        #number of detected objects
        cpm_msg.payload.cpm_containers.perceived_object_container.number_of_perceived_objects = len(msg.detections)

        #list for detected objects
        perceived_objects = []
        #iterate over detected Obejcts
        for i in msg.detections:
            perceived_object = PerceivedObject()

            perceived_object.object_id = i

            #list of all classification results for one object
            classifications = []

            #iterate over all classification results
            for j in msg.detections[i].results:

                classification = ObjectClassWithConfidence()

                #convert detectnet object id to CPM object id
                object_class_id = self.object_class_dict.get(ord(msg.detections[i].results[j].id)+1)
                self.get_logger().info("Object "+str(i+1)+" : "+str(object_class_id))

                #check if object id is known
                if object_class_id is not None:
                    classification.object_class.vehicle_sub_class = object_class_id
                
                else:
                    classification.object_class.vehicle_sub_class = 0 

                #detection confidence from detectnet detections
                classification.confidence = int((msg.detections[i].results[j].score*100))
                classifications.append(classification)

            perceived_object.classification = classifications
            perceived_objects.append(perceived_object)

            #set perceived objects of cpm message
            cpm_msg.payload.cpm_containers.perceived_object_container.perceived_objects = perceived_objects

            #publish the cpm messages
            self.cpm_publisher.publish(cpm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Tranmitter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()