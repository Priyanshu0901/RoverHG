#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pynput.keyboard import Listener
from std_msgs.msg import Int8

class KbNode(Node):

    def __init__(self):
        super().__init__(node_name="Keyboard_Publisher_Node") # type: ignore
        self.get_logger().info("\nKeyboard Input is On \n")
        self.cmd_mov_keyboard_pub = self.create_publisher(Int8, "/direction/keyboard", 2)
        def press_on (key) :
            try:
                self.send_dir_key_commands(key)
            except:
                self.send_stop_key_commands()
        def press_off (key):
            self.send_stop_key_commands()
        with Listener(on_press = press_on, on_release = press_off) as listener: # type: ignore
            listener.join()

    
    def send_dir_key_commands(self,key):
        msg = Int8()
        msg.data = 0
        if key.char == 'w' :
            msg.data = 1
        elif key.char == 'a':
            msg.data = 2
        elif key.char == 's':
            msg.data = 3
        elif key.char == 'd':
            msg.data = 4
        
        self.cmd_mov_keyboard_pub.publish(msg)
        
        self.get_logger().info("\n!!!\tMovement\t!!!!")
    
    def send_stop_key_commands(self):
        msg = Int8()
        msg.data = 0
        self.cmd_mov_keyboard_pub.publish(msg)
        self.get_logger().info("\n!!!!\tMotor Stopped\t!!!!")

        

def main(args = None):
    rclpy.init(args=args)
    node = KbNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()