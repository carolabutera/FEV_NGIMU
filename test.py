
import osc_decoder
import socket
import time
import operations
import numpy as np
import math
import csv
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
from numpy.linalg import norm
from datetime import datetime
from pythonosc import udp_client
import ifcfg
import json
import keyboard
import os  
import calibration 
import PySimpleGUI as sg

def Angle(x): 
    if x[0,0]>0:
        angle=-np.arcsin(x[0,1])*180/3.14
        #print("angle", angle)
    else:
        angle=90-np.arcsin(x[0,1])*180/3.14
        #print("hey", angle)
    return(angle)


#DATA ARE ACQUIRED @50HZ--> first tried with 100Hz but some data were lost

ignore_magnetometer=1# put =1 in case of acquisitions in noisy environment--> NB: align IMUs to each other before running the script!


# BeagleBone Public IP address
for name, interface in ifcfg.interfaces().items():
    if interface['device'] == "wlan0":      # Device name
        IPAddr = interface['inet']          # First IPv4 found
        print("You are connected to the network. IP Address: ", IPAddr)         


send_addresses = ["192.168.0.100","192.168.0.101","192.168.0.102","192.168.0.103","192.168.0.104"] #CHECK-->ultimo è 104 quando aggiungo 5 imu 
send_port = 9000
receive_ports = [8100, 8101, 8102, 8103, 8104]
    

print("Opening UDP socket...")


send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
IPAddr = "192.168.0.105" #CHECK #105 cambiare quando aggiungo 5 imu


for send_address in send_addresses:
    # Change IMUs UDP send addresses to match BeagleBone IP address
    IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
    # Make the led blink
    IMU_client.send_message("/identify", 0.0)
    IMU_client.send_message("/wifi/send/ip", IPAddr) #IP address of the beaglebone (changed with IP address of the computer)
    # if ignore_magnetometer==1:
    #     #IMU_client.send_message("/reset", True)
    #     IMU_client.send_message("/ahrs/magnetometer", False)#true to ignore magnetometer

    if send_address == send_addresses[0]:
        print("Put this IMU on the trunk")
    elif send_address == send_addresses[1]:
        print("Put this IMU on the right upper arm")
    elif send_address == send_addresses[2]:
        print("Put this IMU on the right forearm")
    elif send_address == send_addresses[3]:
        print("Put this IMU on the left upper arm")
    elif send_address == send_addresses[4]:
        print("Put this IMU on the left forearm")
    else:
        print("Error: the send address is not correct")
    time.sleep(2)

# Open the UDP connection to continuously read messages from the IMUs network
receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

index = 0
for receive_socket in receive_sockets:
    receive_socket.bind(("", receive_ports[index]))
    index = index + 1
    receive_socket.setblocking(False)
    
print("Starting communication...")
time.sleep(0.5)
o=0

send_port = 9000
PC_client = udp_client.SimpleUDPClient(send_address, send_port)


# Initialize matrix and vectors in which imu data will be saved 
TO_g=np.identity(3, dtype=float)
UA_R_g=np.identity(3, dtype=float)
FA_R_g=np.identity(3, dtype=float)
UA_L_g=np.identity(3, dtype=float)
FA_L_g=np.identity(3, dtype=float)
a_TO_g,a_UA_R_g,a_FA_R_g,a_UA_L_g,a_FA_L_g=calibration.init_vec()
w_TO_g,w_UA_R_g,w_FA_R_g,w_UA_L_g,w_FA_L_g=calibration.init_vec()
z_onto_xy = np.matrix([[0, 0, 0]])
v = [0, 0, 0]
sum=[0,0,0]
timecount=0
calibration_flag=-1
acquisition_flag=-1
pc=0
rep_n=0
rep_t=0
x_g=np.array([1,0,0]).T
y_g=np.array([0,1,0]).T


while True:

    for udp_socket in receive_sockets: 
        read=False
        while(read == False):
            try:
                data, addr = udp_socket.recvfrom(2048)
                #print("here")            
            except socket.error:
                pass
            else:
                read = True
                for message in osc_decoder.decode(data):
                    #print(message)                
                    time_stamp = message[0]
                    data_type = message[1]              
                    if data_type == '/matrix': #this can be changed with every message avaible from the IMUs (gyro data, temperature...)
                        Rxx = message[2]
                        Ryx = message[3]
                        Rzx = message[4]
                        Rxy = message[5]
                        Ryy = message[6]
                        Rzy = message[7]
                        Rxz = message[8]
                        Ryz = message[9]
                        Rzz = message[10] 
                    
                        if udp_socket.getsockname()[1] == receive_ports[0]:
                            TO_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])                  
                        elif udp_socket.getsockname()[1] == receive_ports[1]:       
                            UA_R_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])    
                        elif udp_socket.getsockname()[1] == receive_ports[2]:
                            FA_R_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])     
                        elif udp_socket.getsockname()[1] == receive_ports[3]:       
                            UA_L_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])    
                        elif udp_socket.getsockname()[1] == receive_ports[4]:
                            FA_L_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])                   
                        else:
                            pass 

                    if data_type =='/earth': #linear accelerations in IMU axis
                        a_x=message[2]
                        a_y=message[3]
                        a_z=message[4] 
                        if udp_socket.getsockname()[1] == receive_ports[0]:
                            a_TO_g=np.array([a_x,a_y,a_z])
                        elif udp_socket.getsockname()[1] == receive_ports[1]:       
                            a_UA_R_g=np.array([a_x,a_y,a_z])
                        elif udp_socket.getsockname()[1] == receive_ports[2]:
                            a_FA_R_g=np.array([a_x,a_y,a_z])
                        elif udp_socket.getsockname()[1] == receive_ports[3]:       
                            a_UA_L_g=np.array([a_x,a_y,a_z])
                        elif udp_socket.getsockname()[1] == receive_ports[4]:
                            a_FA_L_g=np.array([a_x,a_y,a_z])
                        else:
                            pass 
                    if data_type =='/sensors': #angular velocities in IMU axis
                        w_x=message[2]
                        w_y=message[3]
                        w_z=message[4] 
                        if udp_socket.getsockname()[1] == receive_ports[0]:
                            w_TO_g=np.array([w_x,w_y,w_z])
                        elif udp_socket.getsockname()[1] == receive_ports[1]:       
                            w_UA_R_g=np.array([w_x,w_y,w_z])
                        elif udp_socket.getsockname()[1] == receive_ports[2]:
                            w_FA_R_g=np.array([w_x,w_y,w_z])
                        elif udp_socket.getsockname()[1] == receive_ports[3]:       
                            w_UA_L_g=np.array([w_x,w_y,w_z])
                        elif udp_socket.getsockname()[1] == receive_ports[4]:
                            w_FA_L_g=np.array([w_x,w_y,w_z])
                        else:
                            pass 
    TO=TO_g
    UA=UA_R_g
    FA=FA_R_g


    acc=a_TO_g
    acc_onto_y=np.dot([0,1,0], acc, out=None)
    acc_onto_x=np.dot([1,0,0], acc, out=None)

    for i in range(3): 
        v[i] = acc_onto_y.item(0)*y_g.item(i) + acc_onto_x.item(0)*x_g.item(i)  
    acc_onto_xy = np.matrix([[v[0], v[1], v[2]]])
    sum=sum+acc_onto_xy
    TO_UA=np.matmul(TO_g.T,UA_R_g)
    TO_FA=np.matmul(TO_g.T,FA_R_g)
    UA_FA= np.matmul(UA_R_g.T,FA_R_g)
   

    zTO_onto_zUA=np.dot(UA[:,2].T,TO[:,2], out=None)
    zTO_onto_zFA=np.dot(FA[:,2].T,TO[:,2], out=None) 
    # z_onto_y=np.dot(FA[:,1].T,UA[:,2], out=None)
    # z_onto_z=np.dot(FA[:,2].T,UA[:,2], out=None) 

    for i in range(3): 
        v[i] = zTO_onto_zUA.item(0)*UA[:,2].item(i) + zTO_onto_zFA.item(0)*FA[:,2].item(i)  
        #print(v[i])
    zTO_onto_z = np.matrix([[v[0], v[1], v[2]]])
    if zTO_onto_zFA<0:
        HR_1=operations.relative_angle(zTO_onto_z, TO[:,2].T) 
    else: 
        HR_1=math.pi-operations.relative_angle(zTO_onto_z, TO[:,2].T) 


    z_onto_y=np.dot(TO[:,1].T, UA[:,2], out=None) 
    z_onto_x=np.dot(TO[:,0].T, UA[:,2], out=None) 
    vec = [0, 0, 0]
    for i in range(3): 
        vec[i] = z_onto_y.item(0)*TO.item(i,1) + z_onto_x.item(0)*TO.item(i,0)    

    z_onto_xy = np.matrix([[vec[0], vec[1], vec[2]]])

    x_TO=np.array([0,0,0])
    x_TO=TO[:,0]
    

    arm_sign=1
 
        
    if operations.relative_angle(z_onto_xy,TO[:,1].T)<math.pi/2:
        sign=-1*arm_sign
    else:
        sign=+1*arm_sign

    POE = sign*operations.relative_angle(arm_sign*z_onto_xy, -x_TO.T) #right arm
            
    # Angle of elevation 
    AOE = operations.relative_angle(UA[:,2].T,TO[:,2].T) #relative angle btw UA_z  and TO_z

    # Humeral rotation 
    rotPOE = operations.rotZ(POE)#rotation around Z of POE 
    rotAOE = operations.rotY(-arm_sign*AOE) #rotation around  of the AOE  
    rotHR = np.matmul(np.matmul(np.matmul(rotAOE.T,rotPOE.T),TO.T),UA) #shoulder as ZXZ mechanism
    HR = math.atan2(rotHR[1,0],(rotHR[1,1])) #arctg (sin/cos) given that HR is a rotation around z-axis
    

    #correction of the UA rotation matrix according to z-axis of forearm 

    zFA_onto_zUA=norm(np.dot(UA[:,2].T, FA[:,2], out=None))
    z_FA_proj=zFA_onto_zUA*UA[:,2]

    UA_corr=np.identity(3, dtype=float)

    UA_corr_y=z_FA_proj+(-1)*FA[:,2]
    UA_corr_y=UA_corr_y/norm(UA_corr_y)
    UA_corr_x=np.cross(UA_corr_y.T,UA[:,2].T)

    for i in range(0,3):
        UA_corr[i,0]=UA_corr_x[0,i]
        UA_corr[i,1]=UA_corr_y[i]
        UA_corr[i,2]=UA[i,2]

    
    rotHR = np.matmul(np.matmul(np.matmul(rotAOE.T,rotPOE.T),TO.T),UA_corr) #shoulder as ZXZ mechanism
    HR_2 = math.atan2(rotHR[1,0],(rotHR[1,1])) 
    
    
    if (timecount%80==0):



        print("TO", TO_g)
        print("UA", UA_R_g)
        print("FA",FA_R_g)
        print("UA_l", UA_L_g)
        print("FA_l",FA_L_g)

        # print("zTO_onto_zUA",zTO_onto_zUA)
        # print("zTO_onto_zFA",zTO_onto_zFA)
        # print("z_onto_yx" ,zTO_onto_z)

        # print("hr",HR_1*180/3.14)
        # print("hr_2",HR_2*180/3.14)

                


    timecount = timecount+1

