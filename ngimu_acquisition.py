#!/usr/bin/python3

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
import csv_util 
# cambiare assi di convezione isb
#salvare file csv con solo angoli e matrici di rotazione 


#DATA ARE ACQUIRED @50HZ--> first tried with 100Hz but some data were lost

sg.theme("LightGreen2")

output_n=sg.Text(size=(25,1))
output_t=sg.Text(size=(25,1))
output_e=sg.Text(size=(25,1))

layout_calibration = [ 
                [sg.Text("Subject is in N-POSE?", font=('Arial',12))],
                [sg.Button('Start N-POSE calibration', font=('Arial',12)),output_n, sg.Button ('Redo N-POSE', font=('Arial',12))],
                [sg.Text("Calibration type?", font=('Arial',12))],
                [sg.OptionMenu(["UA_tpose", "FA_tpose","ACCELERATION"])],
                [sg.Text("Subject is in T-POSE?", font=('Arial',12))],
                [sg.Button('Start T-POSE calibration', font=('Arial',12)), output_t, sg.Button ('Redo acquisition', font=('Arial',12))],
                [sg.Button("Calibration ok", font=('Arial',12))],
                [sg.Text("NB: wait some seconds before pressing start after changing pose", font=('Arial',12))]]
layout_exercise=[
                [sg.Text("Exercise name?", font=('Arial',12))],
                [sg.Input()],
                [sg.Text("Arm", font=('Arial',12))],
                [sg.OptionMenu(["R","L"])],
                [sg.Button("Start Acquisition", font=('Arial',12)),sg.RButton("Stop Acquisition", font=('Arial',12))],
                [output_e],
                 ]

ignore_magnetometer=0# put =1 in case of acquisitions in noisy environment--> NB: align IMUs to each other before running the script!

def imu_to_isb(mat_imu):
    mat_isb=np.matmul(np.matmul(mat_imu, operations.rotX(math.pi/2)), operations.rotY(math.pi/2))
    return mat_isb

# BeagleBone Public IP address
for name, interface in ifcfg.interfaces().items():
    if interface['device'] == "wlan0":      # Device name
        IPAddr = interface['inet']          # First IPv4 found
        print("You are connected to the network. IP Address: ", IPAddr)         


send_addresses = ["192.168.0.100","192.168.0.101","192.168.0.102"]#,"192.168.0.103","192.168.0.104"] #remove comment when using 5 IMUs 
send_port = 9000
receive_ports = [8100, 8101, 8102]#, 8103, 8104] #remove comment when using 5 IMUs
    

print("Opening UDP socket...")


send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
IPAddr = "192.168.0.105" #CHECK 


for send_address in send_addresses:
    # Change IMUs UDP send addresses to match BeagleBone IP address
    IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
    # Make the led blink
    IMU_client.send_message("/identify", 0.0)
    IMU_client.send_message("/wifi/send/ip", IPAddr) #IP address of the beaglebone (changed with IP address of the computer)
    # if ignore_magnetometer==1:
    #     #IMU_client.send_message("/reset", True)
    #     IMU_client.send_message("/ahrs/magnetometer", False)
    # else: 
    #     IMU_client.send_message("/ahrs/magnetometer", False)
        
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
y_onto_xz = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]
timecount=0
calibration_flag=-1
acquisition_flag=-1
rep_n=0
rep_t=0


window_calibration= sg.Window("Start N-POSE acquisition ", layout_calibration, margins=(200,150))

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
                        a_x_g=message[2]
                        a_y_g=message[3]
                        a_z_g=message[4] 
                        if udp_socket.getsockname()[1] == receive_ports[0]:
                            a_TO_g=np.array([a_x_g,a_y_g,a_z_g])
                        elif udp_socket.getsockname()[1] == receive_ports[1]:       
                           a_UA_R_g=np.array([a_x_g,a_y_g,a_z_g])
                        elif udp_socket.getsockname()[1] == receive_ports[2]:
                            a_FA_R_g=np.array([a_x_g,a_y_g,a_z_g])
                        elif udp_socket.getsockname()[1] == receive_ports[3]:       
                            a_UA_L_g=np.array([a_x_g,a_y_g,a_z_g])
                        elif udp_socket.getsockname()[1] == receive_ports[4]:
                            a_FA_L_g=np.array([a_x_g,a_y_g,a_z_g])
                        else:
                            pass 


    if calibration_flag==-1: #Settings for calibration 
        
        event,values = window_calibration._ReadNonBlocking() #here read


        #window_npose.close()
        if event==('Start N-POSE calibration'):
            output_n.update(value="Acquisition...")
            #output_n.update(value="Acquisition...")
            calibration_flag=0
            sum_TO,sum_UA_R,sum_FA_R,sum_UA_L,sum_FA_L,sum_aTO=calibration.init_rot()
            n=0
            pose="n-pose"
            start=time.time()



    elif calibration_flag==0: #acquisition of npose data
        
        if (time.time()-start<15) and (time.time()-start>10): #N-POSE data acquisiton     
         
            sum_TO,sum_UA_R,sum_FA_R,sum_UA_L,sum_FA_L,sum_aTO=calibration.mat_sum(TO_g,UA_R_g,FA_R_g,UA_L_g,FA_L_g,a_TO_g,sum_TO,sum_UA_R,sum_FA_R,sum_UA_L,sum_FA_L,sum_aTO)
            print("acquisition")

            n=n+1

            csv_util.writer_calib.writerow([pose,rep_n, time.time(),TO_g[0,0],TO_g[0,1],TO_g[0,2],TO_g[1,0],TO_g[1,1],TO_g[1,2],TO_g[2,0],TO_g[2,1],TO_g[2,2],UA_R_g[0,0],UA_R_g[0,1],UA_R_g[0,2],UA_R_g[1,0],UA_R_g[1,1],UA_R_g[1,2],UA_R_g[2,0],UA_R_g[2,1],UA_R_g[2,2],FA_R_g[0,0],FA_R_g[0,1],FA_R_g[0,2],FA_R_g[1,0],FA_R_g[1,1],FA_R_g[1,2],FA_R_g[2,0],FA_R_g[2,1],FA_R_g[2,2],UA_L_g[0,0],UA_L_g[0,1],UA_L_g[0,2],UA_L_g[1,0],UA_L_g[1,1],UA_L_g[1,2],UA_L_g[2,0],UA_L_g[2,1],UA_L_g[2,2],FA_L_g[0,0],FA_L_g[0,1],FA_L_g[0,2],FA_L_g[1,0],FA_L_g[1,1],FA_L_g[1,2],FA_L_g[2,0],FA_L_g[2,1],FA_L_g[2,2],a_TO_g[0],a_TO_g[1],a_TO_g[2],n])


        elif time.time()-start>15: 
            output_n.update(value="DONE!")      
            event,values=window_calibration._ReadNonBlocking()
            calib_type=values[0]

            if event=="Start T-POSE calibration":   
                output_t.update(value="Acquisition")
                TO_npose=sum_TO/n #mean of the matrix in the n-pose
                UA_R_npose=sum_UA_R/n
                FA_R_npose=sum_FA_R/n
                UA_L_npose=sum_UA_L/n
                FA_L_npose=sum_FA_L/n
                a_calib=sum_aTO/n  

                calibration_flag=1
                pose="t-pose"
                rep_t=rep_t+1
                sum_TO,sum_UA_R,sum_FA_R,sum_UA_L,sum_FA_L,sum_aTO=calibration.init_rot()
                n=0
                start=time.time()

            elif event=="Redo N-POSE":
                print("restart npose")
                output_n.update(value="press Start")
                calibration_flag=-1

    elif calibration_flag==1:#acquisition of data during t-pose/walking forward 
        
        if (time.time()-start<15) and (time.time()-start>10):
            
            sum_TO,sum_UA_R,sum_FA_R,sum_UA_L,sum_FA_L,sum_aTO=calibration.mat_sum(TO_g,UA_R_g,FA_R_g,UA_L_g,FA_L_g,a_TO_g, sum_TO,sum_UA_R,sum_FA_R,sum_UA_L,sum_FA_L,sum_aTO)
            print("acquisition")
            n=n+1
            csv_util.writer_calib.writerow([pose,rep_t, time.time(), sum_TO[0,0],sum_TO[0,1],sum_TO[0,2],sum_TO[1,0],sum_TO[1,1],sum_TO[1,2],sum_TO[2,0],sum_TO[2,1],sum_TO[2,2],sum_UA_R[0,0],sum_UA_R[0,1],sum_UA_R[0,2],sum_UA_R[1,0],sum_UA_R[1,1],sum_UA_R[1,2],sum_UA_R[2,0],sum_UA_R[2,1],sum_UA_R[2,2],sum_FA_R[0,0],sum_FA_R[0,1],sum_FA_R[0,2],sum_FA_R[1,0],sum_FA_R[1,1],sum_FA_R[1,2],sum_FA_R[2,0],sum_FA_R[2,1],sum_FA_R[2,2],sum_UA_L[0,0],sum_UA_L[0,1],sum_UA_L[0,2],sum_UA_L[1,0],sum_UA_L[1,1],sum_UA_L[1,2],sum_UA_L[2,0],sum_UA_L[2,1],sum_UA_L[2,2],sum_FA_L[0,0],sum_FA_L[0,1],sum_FA_L[0,2],sum_FA_L[1,0],sum_FA_L[1,1],sum_FA_L[1,2],sum_FA_L[2,0],sum_FA_L[2,1],sum_FA_L[2,2], a_TO_g[0],a_TO_g[1],a_TO_g[2],n])

        elif time.time()-start>15:
            output_t.update(value="DONE!")
            event,values=window_calibration._ReadNonBlocking()
                    
            if event=="Redo acquisition":
                print("restart acquisition")
                sum_TO,sum_UA_R,sum_FA_R,sum_UA_L,sum_FA_L,sum_aTO=calibration.init_rot()
                start=time.time()

            if event==("Calibration ok"):
                UA_R_tpose=sum_UA_R/n #mean of the matrix in the t-pose
                FA_R_tpose=sum_FA_R/n
                UA_L_tpose=sum_UA_L/n
                FA_L_tpose=sum_FA_L/n
 
                a_calib=sum_aTO/n

                window_calibration.close()
                csv_util.calib.close()
                calibration_flag=2
                theta,TO_calib,UA_R_calib,FA_R_calib,UA_L_calib,FA_L_calib=calibration.calib_matrixes(TO_npose,UA_R_npose,FA_R_npose,UA_L_npose,FA_L_npose,UA_R_tpose,FA_R_tpose,a_calib,calib_type)
                window_exercise=sg.Window("Exercises", layout_exercise, margins=(200,150))
                print("theta", theta)

##ACQUISITION 
    elif calibration_flag==2: #calibration is finished: start acquisitions


        if acquisition_flag==-1: #exercise setting                  
            event,values=window_exercise._ReadNonBlocking()

            #creation of the csv file with subject name, exe etc
            
            if event=="Start Acquisition": 

                exe=values[0]
                test_arm=values[1]
                if test_arm=="R":
                    arm=1

                elif test_arm=="L":
                    arm=-1

                file_path_r = os.path.join(csv_util.outdir, csv_util.IDs+"_"+exe+"_"+str(test_arm)+"_R_DATA"+".csv") 
                file_path_l= os.path.join(csv_util.outdir, csv_util.IDs+"_"+exe+"_"+str(test_arm)+"_L_DATA"+".csv") 
                raw_path=os.path.join(csv_util.outdir, csv_util.IDs+"_"+exe+"_"+str(test_arm)+"_RAW.csv")

                file_r=open(file_path_r, 'w', encoding='UTF8', newline='')
                file_l=open(file_path_l, 'w', encoding='UTF8', newline='')
                raw=open(raw_path,'w', encoding='UTF8', newline='')

                writer_fileR=csv.writer(file_r, delimiter=',')
                writer_fileL=csv.writer(file_l,delimiter=',')
                writer_raw=csv.writer(raw, delimiter=',')

                header_fileR=['time','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx', 'UAyx','UAzx','UAxy' ,'UAyy', 'UAzy','UAxz' ,'UAyz' ,'UAzz','FAxx', 'FAyx','FAzx','FAxy' ,'FAyy', 'FAzy','FAxz' ,'FAyz' ,'FAzz','POE','AOE','HR','FE','PS','HR_corr','PS_corr']
                header_fileL=['time','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx', 'UAyx','UAzx','UAxy' ,'UAyy', 'UAzy','UAxz' ,'UAyz' ,'UAzz','FAxx', 'FAyx','FAzx','FAxy' ,'FAyy', 'FAzy','FAxz' ,'FAyz' ,'FAzz','POE','AOE','HR','FE','PS','HR_corr','PS_corr']
                writer_fileR.writerow(header_fileR)
                writer_fileL.writerow(header_fileL)
                header_raw=['time','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx_R', 'UAyx_R','UAzx_R','UAxy_R' ,'UAyy_R', 'UAzy_R','UAxz_R' ,'UAyz_R' ,'UAzz_R','FAxx_R', 'FAyx_R','FAzx_R','FAxy_R' ,'FAyy_R', 'FAzy_R','FAxz_R' ,'FAyz_R' ,'FAzz_R','UAxx_L', 'UAyx_L','UAzx_L','UAxy_L' ,'UAyy_L', 'UAzy_L','UAxz_L' ,'UAyz_L' ,'UAzz_L','FAxx_L', 'FAyx_L','FAzx_L','FAxy_L' ,'FAyy_L', 'FAzy_L','FAxz_L' ,'FAyz_L' ,'FAzz_L']
                writer_raw.writerow(header_raw)

                acquisition_flag=0
                for send_address in send_addresses: 
                    IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
                    IMU_client.send_message("/identify", 0.0)

                start_time=time.time()
                start_date=datetime.now()
                csv_util.writer_subj.writerow(['','','','','', exe,str(test_arm),start_date])

        if acquisition_flag==0: #exercise  acquisitions
            s=0
            event,values=window_exercise._ReadNonBlocking()
            
            #sensor data calibration:    
            TO,UA_R,FA_R,UA_L,FA_L=calibration.calibrate_rot(theta,TO_calib, UA_R_calib,FA_R_calib,UA_L_calib,FA_L_calib,TO_g,UA_R_g,FA_R_g,UA_L_g,FA_L_g)
            
            #transform from IMU to ISB reference frames 
            TO=imu_to_isb(TO)
            UA_R=imu_to_isb(UA_R)
            FA_R=imu_to_isb(FA_R)
            UA_L=imu_to_isb(UA_L)
            FA_L=imu_to_isb(FA_L)


            z_TO=np.array([0,0,0])
            z_TO=TO[:,2]

            # PLANE OF ELEVATION-RIGHT ARM
            y_onto_x=np.dot(TO[:,0].T, UA_R[:,1], out=None) 
            y_onto_z=np.dot(TO[:,2].T, UA_R[:,1], out=None) 
            
            for i in range(3): 
                vec[i] = y_onto_x.item(0)*TO.item(i,0) + y_onto_z.item(0)*TO.item(i,2)    

            y_onto_xz = np.matrix([[vec[0], vec[1], vec[2]]])
                
            if operations.relative_angle(y_onto_xz,TO[:,0].T)<math.pi/2:
                sign=-1
            else:
                sign=+1

            POE_R = sign*operations.relative_angle(y_onto_xz, -z_TO.T) #right arm

            # PLANE OF ELEVATION-LEFT
            y_onto_x=np.dot(TO[:,0].T, UA_L[:,1], out=None) 
            y_onto_z=np.dot(TO[:,2].T, UA_L[:,1], out=None) 
            
            for i in range(3): 
                vec[i] = y_onto_x.item(0)*TO.item(i,0) + y_onto_z.item(0)*TO.item(i,2)    

            y_onto_xz = np.matrix([[vec[0], vec[1], vec[2]]])


            if operations.relative_angle(y_onto_xz,TO[:,0].T)<math.pi/2:
                sign=1
            else:
                sign=-1

            POE_L = sign*operations.relative_angle((-1)*y_onto_xz, -z_TO.T) #left arm
                    
        # ANGLE OF ELEVATION: relative angle btw UA_y  and TO_y
            AOE_R = operations.relative_angle(UA_R[:,1].T,TO[:,1].T) 

            AOE_L= operations.relative_angle(UA_L[:,1].T,TO[:,1].T) 

        # HUMERAL ROTATION: based on the shoulder definition as YXY mechanism
            rotPOE_R= operations.rotY(POE_R)#rotation around Y of POE 
            rotPOE_L= operations.rotY(POE_L) 

            rotAOE_R = operations.rotX((-1)*AOE_R) #rotation around X of the AOE
            rotAOE_L = operations.rotX(AOE_L) 

            rotHR_R = np.matmul(np.matmul(np.matmul(rotAOE_R.T,rotPOE_R.T),TO.T),UA_R) #shoulder as YXY mechanism
            rotHR_L = np.matmul(np.matmul(np.matmul(rotAOE_L.T,rotPOE_L.T),TO.T),UA_L) 

            HR_R = math.atan2(rotHR_R[0,2],(rotHR_R[0,0])) #arctg (sin/cos) given that HR is a rotation around Y #check
            HR_L = math.atan2(rotHR_L[0,2],(rotHR_L[0,0])) #check
            
        # HUMERAL ROTATION WITH UA CORRECTION -RIGHT ARM
            #correction of the UA rotation matrix according to z-axis of forearm 

            yFA_onto_yUA_R=norm(np.dot(UA_R[:,1].T, FA_R[:,1], out=None))
            y_FA_proj_R=yFA_onto_yUA_R*UA_R[:,1]

            UA_corr_R=np.identity(3, dtype=float)

            UA_corr_x=y_FA_proj_R+(-1)*FA_R[:,1]
            UA_corr_x=UA_corr_x/norm(UA_corr_x)
            UA_corr_z=np.cross(UA_corr_x.T,UA_R[:,1].T)

            for i in range(0,3):
                UA_corr_R[i,0]=UA_corr_x[0,i]
                UA_corr_R[i,1]=UA_R[i,1]
                UA_corr_R[i,2]=UA_corr_z[i]
            
            rotHR_corr_R= np.matmul(np.matmul(np.matmul(rotAOE_R.T,rotPOE_R.T),TO.T),UA_corr_R) #shoulder as ZXZ mechanism
            HR_corr_R= math.atan2(rotHR_corr_R[0,2],(rotHR_corr_R[0,0])) 
        
        #HUMERAL ROTATION WITH CORRECTION -LEFT ARM
            yFA_onto_yUA_L=norm(np.dot(UA_L[:,1].T, FA_L[:,1], out=None))
            y_FA_proj_L=yFA_onto_yUA_L*UA_L[:,1]

            UA_corr_L=np.identity(3, dtype=float)

            UA_corr_x=y_FA_proj_L+(-1)*FA_L[:,1]
            UA_corr_x=UA_corr_x/norm(UA_corr_x)
            UA_corr_z=np.cross(UA_corr_x.T,UA_L[:,1].T)

            for i in range(0,3):
                UA_corr_L[i,0]=UA_corr_x[0,i]
                UA_corr_L[i,1]=UA_L[i,1]
                UA_corr_L[i,2]=UA_corr_z[i]
            
            rotHR_corr_L= np.matmul(np.matmul(np.matmul(rotAOE_L.T,rotPOE_L.T),TO.T),UA_corr_L) #shoulder as ZXZ mechanism
            HR_corr_L = math.atan2(rotHR_corr_L[0,2],(rotHR_corr_L[0,0])) 

        # FLEXION EXTENSION -RIGHT
            #FE = operations.relative_angle(FA[:,1].T,UA[:,1.T) #relative angle between y axis
            v=[0,0,0]

            y_onto_x=np.dot(UA_R[:,0].T,FA_R[:,1], out=None)
            y_onto_y=np.dot(UA_R[:,1].T,FA_R[:,1], out=None) 

            for i in range(3): 
                v[i] = y_onto_x.item(0)*UA_R[:,0].item(i) + y_onto_y.item(0)*UA_R[:,1].item(i)  
            y_onto_xy = np.matrix([[v[0], v[1], v[2]]])

            FE_R= operations.relative_angle(y_onto_xy,UA_R[:,1].T) #relative angle between y axis

        #FLEXION EXTENSION-LEFT
            v=[0,0,0]

            y_onto_x=np.dot(UA_L[:,0].T,FA_L[:,1], out=None)
            y_onto_y=np.dot(UA_L[:,1].T,FA_L[:,1], out=None) 

            for i in range(3): 
                v[i] = y_onto_x.item(0)*UA_L[:,0].item(i) + y_onto_y.item(0)*UA_L[:,1].item(i)  
            y_onto_xy = np.matrix([[v[0], v[1], v[2]]])

            FE_L= operations.relative_angle(y_onto_xy,UA_L[:,1].T) #relative angle between z axis



        # PRONOSUPINATION

            rotFE_R=operations.rotZ(FE_R)
            rotFE_L=operations.rotZ(FE_L)

            rotPS_R = np.matmul(np.matmul(rotFE_R.T,UA_R.T),FA_R)
            rotPS_L = np.matmul(np.matmul(rotFE_L.T,UA_L.T),FA_L)

            PS_R = math.atan2(rotPS_R[0,2], rotPS_R[0,0]) #pronosupination is a rotation around z axis
            PS_L = math.atan2(rotPS_L[0,2], rotPS_L[0,0]) 
        # PRONOSUPINATION WITH CORRECTION 
            rotPS_corr_R= np.matmul(np.matmul(rotFE_R.T,UA_corr_R.T),FA_R) 
            rotPS_corr_L= np.matmul(np.matmul(rotFE_L.T,UA_corr_L.T),FA_L) 

            PS_corr_R= math.atan2(rotPS_corr_R[0,2], rotPS_corr_R[0,0])
            PS_corr_L= math.atan2(rotPS_corr_L[0,2], rotPS_corr_L[0,0])
        
        # ANGLE BETWEEN z axes of FA and UA--> could be useful for classification algorithm 
            #AOEF = operations.relative_angle(FA[:,2].T,TO[:,2].T)



            if (AOE_R*180/3.14>155)|(AOE_R*180/3.14<25):
                warning=1
            else:
                warning=0

            #sign adjustment according to ISB standards
            POE_L=(-1)*POE_L
            HR_L=(-1)*HR_L
            PS_L=-(-1)*PS_L
            PS_corr_L=(-1)*PS_corr_L

            #PC_client.send_message("angle", AOE)

            if (timecount%50==0):
                print("RIGHT ARM: ")
                print("POE: ", POE_R*180.0/3.14)                 
                print("AOE: ", AOE_R*180.0/3.14)                
                print("HR: ",HR_R*180.0/3.14)
                print("HR_corr",HR_corr_R*180/3.14)
                print("FE: ",FE_R*180.0/3.14)              
                print("PS: ",PS_R*180.0/3.14)
                print("PS_corr: ",PS_corr_R*180.0/3.14)


                if (AOE_R*180/3.14>155)|(AOE_R*180/3.14<25):
                    print("WARNING! POE and HR for right arm are not accurate")

                print("LEFT ARM: ")
                print("POE: ", POE_L*180.0/3.14)                 
                print("AOE: ", AOE_L*180.0/3.14)                
                print("HR: ",HR_L*180.0/3.14)
                print("HR_corr",HR_corr_L*180/3.14)
                print("FE: ",FE_L*180.0/3.14)              
                print("PS: ",PS_L*180.0/3.14)
                print("PS_corr: ",PS_corr_L*180.0/3.14)
                print("TO",TO)


                if (AOE_L*180/3.14>155)|(AOE_L*180/3.14<25):
                    print("WARNING! POE and HR for left arm are not accurate")

            
            t=time.time()
            data_R=[time_stamp,TO[0,0],TO[0,1],TO[0,2],TO[1,0],TO[1,1],TO[1,2],TO[2,0],TO[2,1],TO[2,2],UA_R[0,0],UA_R[0,1],UA_R[0,2],UA_R[1,0],UA_R[1,1],UA_R[1,2],UA_R[2,0],UA_R[2,1],UA_R[2,2],FA_R[0,0],FA_R[0,1],FA_R[0,2],FA_R[1,0],FA_R[1,1],FA_R[1,2],FA_R[2,0],FA_R[2,1],FA_R[2,2],POE_R*180.0/3.14,AOE_R*180.0/3.14,HR_R*180.0/3.14,FE_R*180.0/3.14,PS_R*180.0/3.14,HR_corr_R*180.0/3.14, PS_corr_R*180.0/3.14]
            data_L=[time_stamp,TO[0,0],TO[0,1],TO[0,2],TO[1,0],TO[1,1],TO[1,2],TO[2,0],TO[2,1],TO[2,2],UA_L[0,0],UA_L[0,1],UA_L[0,2],UA_L[1,0],UA_L[1,1],UA_L[1,2],UA_L[2,0],UA_L[2,1],UA_L[2,2],FA_L[0,0],FA_L[0,1],FA_L[0,2],FA_L[1,0],FA_L[1,1],FA_L[1,2],FA_L[2,0],FA_L[2,1],FA_L[2,2],POE_L*180.0/3.14,AOE_L*180.0/3.14,HR_L*180.0/3.14,FE_L*180.0/3.14,PS_L*180.0/3.14,HR_corr_L*180.0/3.14, PS_corr_L*180.0/3.14]
            writer_fileR.writerow(data_R) 
            writer_fileL.writerow(data_L)  

            timecount = timecount+1


            raw_data=[time_stamp,TO_g[0,0],TO_g[0,1],TO_g[0,2],TO_g[1,0],TO_g[1,1],TO_g[1,2],TO_g[2,0],TO_g[2,1],TO_g[2,2],UA_R_g[0,0],UA_R_g[0,1],UA_R_g[0,2],UA_R_g[1,0],UA_R_g[1,1],UA_R_g[1,2],UA_R_g[2,0],UA_R_g[2,1],UA_R_g[2,2],FA_R_g[0,0],FA_R_g[0,1],FA_R_g[0,2],FA_R_g[1,0],FA_R_g[1,1],FA_R_g[1,2],FA_R_g[2,0],FA_R_g[2,1],FA_R_g[2,2],UA_L_g[0,0],UA_L_g[0,1],UA_L_g[0,2],UA_L_g[1,0],UA_L_g[1,1],UA_L_g[1,2],UA_L_g[2,0],UA_L_g[2,1],UA_L_g[2,2],FA_L_g[0,0],FA_L_g[0,1],FA_L_g[0,2],FA_L_g[1,0],FA_L_g[1,1],FA_L_g[1,2],FA_L_g[2,0],FA_L_g[2,1],FA_L_g[2,2],a_TO_g[0],a_TO_g[1],a_TO_g[2],a_UA_R_g[0],a_UA_R_g[1],a_UA_R_g[2],a_FA_R_g[0],a_FA_R_g[1],a_FA_R_g[2],a_UA_L_g[0],a_UA_L_g[1],a_UA_L_g[2],a_FA_L_g[0],a_FA_L_g[1],a_FA_L_g[2]]
            writer_raw.writerow(raw_data) 

            if event=="Stop Acquisition":          
                IMU_torso=udp_client.SimpleUDPClient(send_addresses[0], send_port)
                IMU_torso.send_message("/identify", 0.0)           
                output_e.update("Acquisition stopped")
                acquisition_flag=-1
                print("Data have been saved")
                file_r.close()
                file_l.close()
                raw.close()



    
            