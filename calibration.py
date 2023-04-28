import numpy as np
import operations
import time
import math

import os
import csv

##NB: calibration is done wrt to IMU reference frames (z axis up)

def mat_sum(to,ua_r,fa_r,ua_l,fa_l,a_to,to_sum,ua_r_sum,fa_r_sum,ua_l_sum,fa_l_sum,a_to_sum): 
    to_sum=to_sum+to  #to store all the matrix in the n-pose
    ua_r_sum=ua_r_sum+ua_r          
    fa_r_sum=fa_r_sum+fa_r
    ua_l_sum=ua_l_sum+ua_l         
    fa_l_sum=fa_l_sum+fa_l
    a_to_sum=a_to_sum+a_to

           
    return to_sum,ua_r_sum,fa_r_sum,ua_l_sum,fa_l_sum,a_to_sum

def init_rot():
    to=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    ua_r=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    fa_r=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    ua_l=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    fa_l=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    a_to=[0,0,0]
    return(to,ua_r,fa_r,ua_l,fa_l,a_to)

def init_vec():
    to=[0,0,0]
    ua_r=[0,0,0]
    fa_r=[0,0,0]
    ua_l=[0,0,0]
    fa_l=[0,0,0]
    return(to,ua_r,fa_r,ua_l,fa_l)
    

def calib_matrixes(to_npose,ua_r_npose,fa_r_npose,ua_l_npose,fa_l_npose,ua_r_tpose,fa_r_tpose,acc,c_type):
    v=[0,0,0]
    x_g=np.array([1,0,0]).T
    y_g=np.array([0,1,0]).T

    if (c_type=="UA_tpose") or (c_type=="FA_tpose"): 
        if (c_type=="UA_tpose"):
            print("calibration with UA tpose")
            tpose=ua_r_tpose
            npose=ua_r_npose
        else: 
            print("calibration with FA tpose")
            tpose=fa_r_tpose
            npose=fa_r_npose
        tpose_calib=np.matmul(tpose, npose.T)
        z_onto_y=np.dot([0,1,0], tpose_calib[:,2], out=None)
        z_onto_x=np.dot([1,0,0], tpose_calib[:,2], out=None) 

        for i in range(3): 
            v[i] = z_onto_y.item(0)*y_g.item(i) + z_onto_x.item(0)*x_g.item(i)  
        z_onto_xy = np.matrix([[v[0], v[1], v[2]]])

        alpha=operations.relative_angle((-1)*z_onto_xy,[0,1,0])

        if alpha < math.pi/2:   
            theta=operations.relative_angle((-1)*z_onto_xy , [1,0,0])

        else:
            theta=2*math.pi-operations.relative_angle((-1)*z_onto_xy , [1,0,0]) #calibration with UA
    
    elif c_type=="ACCELERATION": 
        print("calibration with walking acceleration ")
        acc_onto_x=np.dot([1,0,0], acc, out=None)
        acc_onto_y=np.dot([0,1,0], acc, out=None)

        for i in range(3): 
            v[i] = acc_onto_y.item(0)*y_g.item(i) + acc_onto_x.item(0)*x_g.item(i)  
        acc_onto_xy = np.matrix([[v[0], v[1], v[2]]])
        print("acc_onto_xy",acc_onto_xy)

        alpha=operations.relative_angle(acc_onto_xy,[1,0,0])

        if alpha > math.pi/2:   
            theta=operations.relative_angle(acc_onto_xy,[0,1,0])

        else:
            theta=2*math.pi-operations.relative_angle(acc_onto_xy,[0,1,0]) #calibration with UA


    #matrix bewteen n-pose and body reference frame
    to_calib=np.matmul(operations.rotZ(theta).T, to_npose)
    ua_r_calib=np.matmul(operations.rotZ(theta).T, ua_r_npose)
    fa_r_calib=np.matmul(operations.rotZ(theta).T, fa_r_npose)
    ua_l_calib=np.matmul(operations.rotZ(theta).T, ua_l_npose)
    fa_l_calib=np.matmul(operations.rotZ(theta).T, fa_l_npose)
    
    
    return theta,to_calib,ua_r_calib,fa_r_calib,ua_l_calib,fa_l_calib


def calibrate_rot(theta,to_calib, ua_r_calib,fa_r_calib,ua_l_calib,fa_l_calib,to_g,ua_r_g,fa_r_g,ua_l_g,fa_l_g):
    TO_b=np.matmul(operations.rotZ(theta).T,to_g)#Transform to move y-axis perpendicular to the torso                
    UA_R_b=np.matmul(operations.rotZ(theta).T,ua_r_g)
    FA_R_b=np.matmul(operations.rotZ(theta).T,fa_r_g)
    UA_L_b=np.matmul(operations.rotZ(theta).T,ua_l_g)
    FA_L_b=np.matmul(operations.rotZ(theta).T,fa_l_g)

    #Calibrated matrix (wrt to calibrated NPOSE initial position) expressed wrt to BODY reference frame
    to=np.matmul(TO_b, to_calib.T)  
    ua_r=np.matmul(UA_R_b, ua_r_calib.T)
    fa_r=np.matmul(FA_R_b, fa_r_calib.T)
    ua_l=np.matmul(UA_L_b, ua_l_calib.T)
    fa_l=np.matmul(FA_L_b, fa_l_calib.T)
    

    return to,ua_r,fa_r,ua_l,fa_l


#to calibrate accelerations and angular velocities 
def calibrate_dot(to_calib, ua_r_calib,fa_r_calib,ua_l_calib,fa_l_calib,to,ua_r,fa_r,ua_l,fa_l): #cambiare con nuova calibrazione

    to_b=np.matmul(to_calib,to)
    ua_r_b=np.matmul(ua_r_calib,ua_r)
    fa_r_b=np.matmul(fa_r_calib,fa_r)
    ua_l_b=np.matmul(ua_l_calib,ua_l)
    fa_l_b=np.matmul(fa_l_calib,fa_l)

    return to_b,ua_r_b,fa_r_b,ua_l_b,fa_l_b








