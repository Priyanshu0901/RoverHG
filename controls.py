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

def move(lm,rm,st):
    the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,lm,st,0,0,0,0,0,0)
    the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,rm,st,0,0,0,0,0,0)
    msg = the_connection.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)

def f_t():
    move(1,3,1)
    
def f_f():
    move(1,3,0)
        
def b_t():
    move(2,4,1)

def b_f():
    move(2,4,0)
    
def l_t():
    move(2,3,1)

def l_f():
    move(2,3,0)

def r_t():
    move(1,4,1)

def r_f():
    move(1,4,0)

f_t()
time.sleep(5)
f_f()
b_t()
time.sleep(5)
b_f()
l_t()
time.sleep(5)
l_f()
r_t()
time.sleep(5)
r_f()