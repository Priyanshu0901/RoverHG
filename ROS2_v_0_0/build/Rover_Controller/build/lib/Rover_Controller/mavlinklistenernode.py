#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil


class MLisNode(Node):

    def __init__(self,the_connection):
        super().__init__(node_name="Mavlink_Listener_Node") # type: ignore
        msg = the_connection.recv_match(type='ATTITUDE',blocking=True)
        self.get_logger().info("MavLink :")
        print(msg)
        

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