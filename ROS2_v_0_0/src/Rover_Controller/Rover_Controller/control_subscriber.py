#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Int8

class ControlSubscriberNode(Node):

    def __init__(self,the_connection):
        super().__init__(node_name="Control_Subscriber_Node") # type: ignore
        self.MavC = the_connection
        self.control_subcriber_ = self.create_subscription(Int8,"/direction/keyboard", self.control_callback , 2)

    def move(self,lm,rm,st):
        self.MavC.mav.command_long_send(self.MavC.target_system,self.MavC.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,lm,st,0,0,0,0,0,0)
        self.MavC.mav.command_long_send(self.MavC.target_system,self.MavC.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,rm,st,0,0,0,0,0,0)
        msg = self.MavC.recv_match(type='COMMAND_ACK',blocking=True)
        self.get_logger().info(str(msg))

    def m_stop(self):
        self.MavC.mav.command_long_send(self.MavC.target_system,self.MavC.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,1,0,0,0,0,0,0,0)
        self.MavC.mav.command_long_send(self.MavC.target_system,self.MavC.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,2,0,0,0,0,0,0,0)
        self.MavC.mav.command_long_send(self.MavC.target_system,self.MavC.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,3,0,0,0,0,0,0,0)
        self.MavC.mav.command_long_send(self.MavC.target_system,self.MavC.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,4,0,0,0,0,0,0,0)
        msg = self.MavC.recv_match(type='COMMAND_ACK',blocking=True)
        self.get_logger().info(str(msg))
        self.get_logger().info("\n!!!!Motors Stopped!!!\n")

    def f_t(self):
        self.move(1,3,1)
        self.get_logger().info("\n!!!!Motors Forward!!!\n")
        
    def b_t(self):
        self.move(2,4,1)
        self.get_logger().info("\n!!!!Motors Back!!!\n")
    
    def l_t(self):
        self.move(2,3,1)
        self.get_logger().info("\n!!!!Motors Left!!!\n")

    def r_t(self):
        self.move(1,4,1)
        self.get_logger().info("\n!!!!Motors Right!!!\n")      
    
    def control_callback(self,msg : Int8):
        self.get_logger().info(str(msg))
        if msg.data == 1:
            self.f_t()
                #direc = 1
        elif msg.data == 2:
            self.l_t()
                #direc = 2
        elif msg.data == 3:
            self.b_t()
                #direc = 3
        elif msg.data == 4:
            self.r_t()
                #direc = 4
        elif msg.data == 0:
            self.m_stop()

        

def main(args = None):
    rclpy.init(args=args)
    the_connection = mavutil.mavlink_connection('/dev/ttyACM0')
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
    node = ControlSubscriberNode(the_connection)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()