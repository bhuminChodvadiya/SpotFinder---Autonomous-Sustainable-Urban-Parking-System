uint8 object_id         # id of the detected Objects

uint8 object_class      # class of the detected Object
				#0 unknown          - information about traffic participant is not provided,
				#1 pedestrian       - human being not using a mechanical device for their trip (VRU profile 1),
				#2 cyclist          - non-motorized unicycles, bicycles , tricycles, quadracycles (VRU profile 2),
				#3 moped            - light motor vehicles with less than four wheels as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class L1, L2 (VRU Profile 3),
				#4 motorcycles      - motor vehicles with less than four wheels as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class L3, L4, L5, L6, L7 (VRU Profile 3),
				#5 passengerCar     - small passenger vehicles as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class M1,
				#6 bus              - large passenger vehicles as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class M2, M3,
				#7 lightTruck       - light Goods Vehicles as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class N1,
				#8 heavyTruck       - Heavy Goods Vehicles as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class N2 and N3,
				#9 trailer          - unpowered vehicle that is intended to be towed by a powered vehicle as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class O,
				#10 specialVehicles - vehicles which have special purposes other than the above (e.g. moving road works vehicle),
				#11 tram            - vehicle which runs on tracks along public streets,
				#12 lightVruVehicle - human being traveling on light vehicle, incl. possible use of roller skates or skateboards (VRU profile 2),
				#13 animal          - animal presenting a safety risk to other road users e.g. domesticated dog in a city or horse (VRU Profile 4),-
				#14 agricultural    - agricultural and forestry vehicles as defined in UNECE/TRANS/WP.29/78/Rev.4 [16] class T,
				#15 roadSideUnit    - infrastructure typically positioned outside of the drivable roadway (e.g. on a gantry, on a pole, on a stationary road works trailer); the infrastructure is static during the entire operation period of the ITS-S (e.g. no stop and go activity),
				#16-255             - are reserved for future usage.

uint8 confidence         # confidence of the detected Object
                # values from 0 to 100
                # unit percentage

float32 x_position        # x coordinate of the detected Object in the ego vehicle coordination system
                # unit meters
    
float32 y_position        # y coordinate of the detected Object in the ego vehicle coordination system
                # unit meters

float32 z_position        # z coordinate of the detected Object in the ego vehicle coordination system
                # unit meters

float32 yaw_angle		  # yaw angle of the detected Object compared with the heading of the ego vehicle		  
				# unit degree

float32 width             # width of the detected Object
                # unit meters

float32 length            # length of the detected Object
                # unit meters


