import numpy as np
import math
from numpy.linalg import norm

def relative_angle(v1,v2):
    angle_rel = math.atan2(norm(np.cross(v1,v2),1),(np.dot(v1,np.transpose(v2))))
    return angle_rel

def rotX(alpha):  #rotation around x axis
    R=np.matrix([[1,0,0],
                 [0,math.cos(alpha),-math.sin(alpha)],
                 [0,math.sin(alpha),math.cos(alpha)]])
    return R

def rotY(alpha): #rotation around y axis 
    R=np.matrix([[math.cos(alpha),0,math.sin(alpha)],
                 [0,1,0],
                 [-math.sin(alpha),0,math.cos(alpha)]])
    return R

def rotZ(alpha): #rotation arounf z axis
    R=np.matrix([[math.cos(alpha),-math.sin(alpha),0],
                 [math.sin(alpha),math.cos(alpha),0],
                 [0,0,1]])
    return R

def rotZ_T_rotX(alpha):
    R=np.matrix([[math.cos(alpha)*math.cos(alpha), math.sin(alpha), math.sin(alpha)*math.cos(alpha)],
                [-math.sin(alpha)*math.cos(alpha),math.cos(alpha),-math.sin(alpha)*math.sin(alpha)],
                [-math.sin(alpha),0,math.cos(alpha)]])

