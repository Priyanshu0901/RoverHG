import pygame
from pymavlink import mavutil

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

def m_stop():
    the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,1,0,0,0,0,0,0,0)
    the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,2,0,0,0,0,0,0,0)
    the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,3,0,0,0,0,0,0,0)
    the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,4,0,0,0,0,0,0,0)
    msg = the_connection.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)
    print("\n!!!!Motors stopped!!!\n")

def f_t():
    move(1,3,1)
    print("\n!!!!Motors Forward!!!\n")
        
def b_t():
    move(2,4,1)
    print("\n!!!!Motors Back!!!\n")
    
def l_t():
    move(2,3,1)
    print("\n!!!!Motors Left!!!\n")

def r_t():
    move(1,4,1)
    print("\n!!!!Motors Right!!!\n")

pygame.init()

screen = pygame.display.set_mode((600,400))

#direc = 0
run = True

while run:    

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                f_t()
                #direc = 1
            elif event.key == pygame.K_a:
                l_t()
                #direc = 2
            elif event.key == pygame.K_s:
                b_t()
                #direc = 3
            elif event.key == pygame.K_d:
                r_t()
                #direc = 4
            elif event.key == pygame.K_p:
                run = False
        if event.type == pygame.KEYUP:
            m_stop()
            #direc = 0

    '''
    key = pygame.key.get_pressed()
    if key[pygame.K_w] == True:
        direc = 1
    elif key[pygame.K_a] == True:
        direc = 2
    elif key[pygame.K_s] == True:
        direc = 3
    elif key[pygame.K_d] == True:
        direc = 4
    elif key[pygame.K_p] == True:
        run = False
    else:
        direc = 0
    '''

    #print(direc)

pygame.quit()