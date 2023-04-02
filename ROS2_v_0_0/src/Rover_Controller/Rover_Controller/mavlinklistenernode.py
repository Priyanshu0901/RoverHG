#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil


class MLisNode(Node):

    def __init__(self,the_connection):
        super().__init__(node_name="Mavlink_Listener_Node") # type: ignore
        self.MavC = the_connection
        self.create_timer(1.0,self.timer_callback)        
    
    def timer_callback(self):
        try: 
            Latitude = self.MavC.messages['GLOBAL_POSITION_INT'].lat
            Longitude = self.MavC.messages['GLOBAL_POSITION_INT'].lon
            altitude = self.MavC.messages['GLOBAL_POSITION_INT'].relative_alt
            timestamp = self.MavC.time_since('GPS_RAW_INT')
            self.get_logger().info("\nMavLink :" + 
                                   "\n\tTime Stamp: \t" + str(timestamp) +
                                   "\n\tLatitude: \t" + str(Latitude) + "\tdegE7" + 
                                   "\n\tLongitude: \t" + str(Longitude) + "\tdegE7" + 
                                   " \n\tAltitude: \t" + str(altitude) + "\tmm" 
                                   )
        except:
            self.get_logger().info("\nMavLink : \n\t!!!!\tNo GPS_RAW_INT message received\t!!!!")  #self.MavC.recv_match(type='ATTITUDE',blocking=True)
        

def main(args = None):
    rclpy.init(args=args)
    the_connection = mavutil.mavlink_connection('/dev/ttyACM0')
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
    node = MLisNode(the_connection)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()