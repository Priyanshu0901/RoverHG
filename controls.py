'''
MA1 = RELAY 1 
MA2 = RELAY 2
MB1 = RELAY 3
MB2 = RELAY 4

ENA = SERVO 1
ENB = SERVO 2
'''

from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('/dev/ttyACM0')

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,1,1,0,0,0,0,0,0)

msg = the_connection.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

time.sleep(10)

the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,1,0,0,0,0,0,0,0) 

msg = the_connection.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)