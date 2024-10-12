float32 x_coordinate        # x coordinate of the vehicle in model city coordinates

float32 y_coordinate        # y coordinate of the vehicle in model city coordinates

float32 yaw_angle		  # yaw angle of the detected Object compared with the heading of the ego vehicle		  
				# 0 -90 -180 - 90 | vehicle facing top right bottom left


float32 velocity            # velocity of the vehicle in kph

float32 acceleration        # acceleration of the vehicle in m/s²

int32 latitude              #This DE represents the absolute geographical latitude in a WGS84 coordinate system, providing a 
                            #range of 90 degrees in north orin south hemisphere. The specific WGS84 coordinate system is 
                            #specified by the corresponding standards applying this DE.
                            #n (n ≥ -900000000 and n < 0) x 10^-7 degree, i.e. negative values for latitudes south of the Equator,
                            #0 is used for the latitude of the equator,
                            #n (n > 0 and n < 900000001) x 10^-7 degree, i.e. positive values for latitudes north of the Equator,
                            #900000001 when the information is unavailable.
                            #Unit: 10^-7 degree

int32 longitude             #This DE represents the absolute geographical longitude in a WGS84 coordinate system, providing a 
                            #range of 180 degreesto the east or to the west of the prime meridian. The specific WGS84 coordinate 
                            #system is specified by the corresponding standards applying this DE.
                            #n (n > -1800000000 and n < 0) x 10^-7 degree, i.e. negative values for longitudes to the west,
                            #0 to indicate the prime meridian,
                            #n (n > 0 and n < 1800000001) x 10^-7 degree, i.e. positive values for longitudes to the east,
                            #1800000001 when the information is unavailable.
                            #The value -1800000000 shall not be used.
                            #Unit: 10^-7 degree