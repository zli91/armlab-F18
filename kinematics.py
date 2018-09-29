import numpy as np
from math import *
#expm is a matrix exponential function
from scipy.linalg import expm
np.set_printoptions(precision=5)
import matplotlib.pyplot as plt

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""
input = [[1],[2],[3],[4]]


def FK_dh(joint_angles, link):
    if joint_angles == 0:
        return
    th1,joint_angles=np.split(joint_angles,[1])
    th2,joint_angles=np.split(joint_angles,[1])
    th3,th4=np.split(joint_angles,[1])
    th1 = float(th1)
    th2 = float(th2)
    th3 = float(th3)
    th4 = float(th4)
    
    T1 = [
         [ cos(th1),   0,      sin(th1),     0],
         [ sin(th1),   0, -1.0*cos(th1),     0],
         [        0, 1.0,             0, 118.0],
         [        0,   0,             0,   1.0]
         ]

    
    T2 = [
         [ cos(th1)*cos(th2), -1.0*cos(th1)*sin(th2),      sin(th1), 99.0*cos(th1)*cos(th2)],
         [ cos(th2)*sin(th1), -1.0*sin(th1)*sin(th2), -1.0*cos(th1), 99.0*cos(th2)*sin(th1)],
         [          sin(th2),               cos(th2),             0,  99.0*sin(th2) + 118.0],
         [                 0,                      0,             0,                    1.0]
         ]
    
    T3 = [
         [ cos(th1)*cos(th2)*cos(th3) - 1.0*cos(th1)*sin(th2)*sin(th3), - 1.0*cos(th1)*cos(th2)*sin(th3) - 1.0*cos(th1)*cos(th3)*sin(th2),      sin(th1), 99.0*cos(th1)*cos(th2) - 99.0*cos(th1)*sin(th2)*sin(th3) + 99.0*cos(th1)*cos(th2)*cos(th3)],
         [ cos(th2)*cos(th3)*sin(th1) - 1.0*sin(th1)*sin(th2)*sin(th3), - 1.0*cos(th2)*sin(th1)*sin(th3) - 1.0*cos(th3)*sin(th1)*sin(th2), -1.0*cos(th1), 99.0*cos(th2)*sin(th1) - 99.0*sin(th1)*sin(th2)*sin(th3) + 99.0*cos(th2)*cos(th3)*sin(th1)],
         [                       cos(th2)*sin(th3) + cos(th3)*sin(th2),                         cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3),             0,                    99.0*sin(th2) + 99.0*cos(th2)*sin(th3) + 99.0*cos(th3)*sin(th2) + 118.0],
         [                                                           0,                                                                 0,             0,                                                                                        1.0]
         ]
        
    T4 = [
         [ - 1.0*cos(th4)*(cos(th1)*sin(th2)*sin(th3) - 1.0*cos(th1)*cos(th2)*cos(th3)) - 1.0*sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)), sin(th4)*(cos(th1)*sin(th2)*sin(th3) - 1.0*cos(th1)*cos(th2)*cos(th3)) - 1.0*cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)),      sin(th1), 99.0*cos(th1)*cos(th2) - 143.6*cos(th4)*(cos(th1)*sin(th2)*sin(th3) - 1.0*cos(th1)*cos(th2)*cos(th3)) - 143.6*sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - 99.0*cos(th1)*sin(th2)*sin(th3) + 99.0*cos(th1)*cos(th2)*cos(th3)],
         [ - 1.0*cos(th4)*(sin(th1)*sin(th2)*sin(th3) - 1.0*cos(th2)*cos(th3)*sin(th1)) - 1.0*sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)), sin(th4)*(sin(th1)*sin(th2)*sin(th3) - 1.0*cos(th2)*cos(th3)*sin(th1)) - 1.0*cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)), -1.0*cos(th1), 99.0*cos(th2)*sin(th1) - 143.6*cos(th4)*(sin(th1)*sin(th2)*sin(th3) - 1.0*cos(th2)*cos(th3)*sin(th1)) - 143.6*sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - 99.0*sin(th1)*sin(th2)*sin(th3) + 99.0*cos(th2)*cos(th3)*sin(th1)],
         [                                               sin(th4)*(cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3)) + cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)),                                     cos(th4)*(cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3)) - 1.0*sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)),             0,                                                        99.0*sin(th2) + 99.0*cos(th2)*sin(th3) + 99.0*cos(th3)*sin(th2) + 143.6*sin(th4)*(cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3)) + 143.6*cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + 118.0],
         [                                                                                                                                                     0,                                                                                                                                               0,             0,                                                                                                                                                                                                                                                  1.0]
         ]
    FK4 = np.round([T1,T2,T3,T4],6)
    #FK4 = format([T1,T2,T3,T4], '.2f')
    #FK4 = ([T1,T2,T3,T4])
    #print 'FK_DH result:\n',(np.matrix(FK4[link-1]))
    print  (np.matrix(FK4[link-1])) 
    return np.matrix(FK4[link-1])

    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    pass

def FK_pox(joint_angles):
    """

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    l1 = 118;
    l2 = 99;
    l3 = 99;
    l4 = 143.6;
    w1 = np.array([[0, -1, 0],[1,0,0],[0,0,0]]) # rotation in z-axis
    w2 = np.array([[0, 0, 1],[0,0,0],[-1,0,0]]) # rotation in y-axis

    pass

def IK(pose):
    
    d1 = 118
    a2 = 99
    a3 = 99
    a4 = 143.6
    Xe,pose=np.split(pose,[1])
    Ye,pose=np.split(pose,[1])
    Ze,phi=np.split(pose,[1])
    if phi>0:
        print 'error: phi must be negative according to convention'
        return 0
    Xe = float(Xe)
    Ye = float(Ye)
    Ze = float(Ze) - d1
    phi = float(phi)
    
    #parameters:
    
    Re = (Xe**2 + Ye**2)**0.5
    print 'Re:',Re
    if (Re>341.6 or Ze>118+341.6):
        print 'error: location out of range'
        return 0
    elif Re>a2+a3+a4*cos(-phi):
        print 'error: euler angle too stiff'
        return 0
    dR = Re - a4*cos(phi)
    print 'dR:',dR
    dZ = Ze + a4*sin(-phi)
    print 'dZ',dZ
    beta = atan2(dZ,dR)
    print 'beta:',beta
    th1 = atan2(Ye,Xe)
    print 'th1:',th1
    print 'dRdZ:' ,(dZ**2+dR**2)**0.5
    th3 = -1*acos(round((dZ**2+dR**2-a2**2-a3**2)/(2*a2*a3),6))
    print 'th3:',th3
    alpha = atan2(a3*sin(-th3),a2+a3*cos(-th3))
    print 'alpha:',alpha
    th2 = beta+alpha
    print 'th2:',th2
    th4 = phi - th2 - th3
    
    print 'IK result:',[th1,th2,th3,th4]
    Xs = [0,a2*cos(th2),a2*cos(th2)+a3*cos(th2+th3),a2*cos(th2)+a3*cos(th2+th3)+a4*cos(phi)]
    Ys = [0,a2*sin(th2),a2*sin(th2)+a3*sin(th2+th3),a2*sin(th2)+a3*sin(th2+th3)+a4*sin(phi)]
    plt.plot(Xs,Ys,'-o')
    plt.title('plot for X Y Z phi:' )
    plt.show()
    return[th1,th2,th3,th4]
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    pass


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass

FK_dh(IK([0,100,100,-pi/2]),4)