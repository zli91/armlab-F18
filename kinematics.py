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
x_off = 304  # distances from center of the bottom of ReArm to world origin
y_off = 301.5

def FK_dh(joint_angles, link):
    if joint_angles == 0:
        return
    th1,joint_angles=np.split(joint_angles,[1])
    th2,joint_angles=np.split(joint_angles,[1])
    th3,th4=np.split(joint_angles,[1])
    th1 = float(th1)
    th2 = float(th2)+pi/2
    th3 = float(th3)
    th4 = float(th4)

    offset_mat = np.array([[0,0,0,y_off],[0,0,0,x_off],[0,0,0,0],[0,0,0,0]])
    T1 = [
         [ cos(th1),   0,      sin(th1),     0],
         [ sin(th1),   0, -1.0*cos(th1),     0],
         [        0, 1.0,             0, 118.0],
         [        0,   0,             0,   1.0]
         ]+offset_mat

    
    T2 = [
         [ cos(th1)*cos(th2), -1.0*cos(th1)*sin(th2),      sin(th1), 99.0*cos(th1)*cos(th2)],
         [ cos(th2)*sin(th1), -1.0*sin(th1)*sin(th2), -1.0*cos(th1), 99.0*cos(th2)*sin(th1)],
         [          sin(th2),               cos(th2),             0,  99.0*sin(th2) + 118.0],
         [                 0,                      0,             0,                    1.0]
         ]+offset_mat
    
    T3 = [
         [ cos(th1)*cos(th2)*cos(th3) - 1.0*cos(th1)*sin(th2)*sin(th3), - 1.0*cos(th1)*cos(th2)*sin(th3) - 1.0*cos(th1)*cos(th3)*sin(th2),      sin(th1), 99.0*cos(th1)*cos(th2) - 99.0*cos(th1)*sin(th2)*sin(th3) + 99.0*cos(th1)*cos(th2)*cos(th3)],
         [ cos(th2)*cos(th3)*sin(th1) - 1.0*sin(th1)*sin(th2)*sin(th3), - 1.0*cos(th2)*sin(th1)*sin(th3) - 1.0*cos(th3)*sin(th1)*sin(th2), -1.0*cos(th1), 99.0*cos(th2)*sin(th1) - 99.0*sin(th1)*sin(th2)*sin(th3) + 99.0*cos(th2)*cos(th3)*sin(th1)],
         [                       cos(th2)*sin(th3) + cos(th3)*sin(th2),                         cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3),             0,                    99.0*sin(th2) + 99.0*cos(th2)*sin(th3) + 99.0*cos(th3)*sin(th2) + 118.0],
         [                                                           0,                                                                 0,             0,                                                                                        1.0]
         ]+offset_mat
        
    T4 = [
         [ - 1.0*cos(th4)*(cos(th1)*sin(th2)*sin(th3) - 1.0*cos(th1)*cos(th2)*cos(th3)) - 1.0*sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)), sin(th4)*(cos(th1)*sin(th2)*sin(th3) - 1.0*cos(th1)*cos(th2)*cos(th3)) - 1.0*cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)),      sin(th1), 99.0*cos(th1)*cos(th2) - 110*cos(th4)*(cos(th1)*sin(th2)*sin(th3) - 1.0*cos(th1)*cos(th2)*cos(th3)) - 110*sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - 99.0*cos(th1)*sin(th2)*sin(th3) + 99.0*cos(th1)*cos(th2)*cos(th3)],
         [ - 1.0*cos(th4)*(sin(th1)*sin(th2)*sin(th3) - 1.0*cos(th2)*cos(th3)*sin(th1)) - 1.0*sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)), sin(th4)*(sin(th1)*sin(th2)*sin(th3) - 1.0*cos(th2)*cos(th3)*sin(th1)) - 1.0*cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)), -1.0*cos(th1), 99.0*cos(th2)*sin(th1) - 110*cos(th4)*(sin(th1)*sin(th2)*sin(th3) - 1.0*cos(th2)*cos(th3)*sin(th1)) - 110*sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - 99.0*sin(th1)*sin(th2)*sin(th3) + 99.0*cos(th2)*cos(th3)*sin(th1)],
         [                                               sin(th4)*(cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3)) + cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)),                                     cos(th4)*(cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3)) - 1.0*sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)),             0,                                                        99.0*sin(th2) + 99.0*cos(th2)*sin(th3) + 99.0*cos(th3)*sin(th2) + 110*sin(th4)*(cos(th2)*cos(th3) - 1.0*sin(th2)*sin(th3)) + 110*cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + 118.0],
         [                                                                                                                                                     0,                                                                                                                                               0,             0,                                                                                                                                                                                                                                                  1.0]
         ]+offset_mat
    FK4 = np.round([T1,T2,T3,T4],6)
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
    #print joint_angles
    l1 = 118;   # lengths of links in mm
    l2 = 99;
    l3 = 99;
    l4 = 143.6;
    # matrix changing tool frame to world frame
    # set the world frame here centered at the bottom center of the base
    M = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,l1+l2+l3+l4],[0,0,0,1]])
    # angular velocities
    wz = np.array([[0, -1, 0],[1,0,0],[0,0,0]]) # rotation in z-axis
    wzv = np.array([[0],[0],[1]])
    wy = np.array([[0, 0, 1],[0,0,0],[-1,0,0]]) # rotation in y-axis
    wyv = np.array([[0],[1],[0]])
    zeros = np.array([[0,0,0,1]])
    # linear velocities
    v = [np.array([[0],[0],[0]]),np.array([[0-l1],[0],[0]]),np.array([[0-l1-l2],[0],[0]]),np.array([[0-l1-l2-l3],[0],[0]])]
    s = [];
    # compute exponential matrices using equations listed in https://en.wikipedia.org/wiki/Product_of_exponentials_formula
    # e: e^(omega,theta); t: t; s: the twist
    # s[0]: twist for the base
    e = np.identity(3) + wz*sin(joint_angles[0]) + np.linalg.matrix_power(wz, 2)*(1-cos(joint_angles[0]))
    wzv_13 = wzv.reshape(1,3);
    v_13 = v[0].reshape(1,3);
    cross = np.cross(wzv_13,v_13).reshape(3,1)
    t = np.dot((np.identity(3)-e),cross) + np.dot(np.dot(wzv,wzv_13),v[0])*joint_angles[0]
    s.append(np.concatenate(((np.concatenate((e,t.reshape(3,1)),axis=1)),zeros),axis=0))

    # s[1:3]: twist for should, elbow, and wrist
    for i in range(1,4):
        e = np.identity(3) + wy*sin(joint_angles[i]) + np.linalg.matrix_power(wy, 2)*(1-cos(joint_angles[i]))
        wyv_13 = wyv.reshape(1,3);
        v_13 = v[i].reshape(1,3);
        cross = np.cross(wyv_13,v_13).reshape(3,1)
        t = np.dot((np.identity(3)-e),cross) + np.dot(np.dot(wyv,wyv_13),v[i])*joint_angles[i]
        s.append(np.concatenate(((np.concatenate((e,t.reshape(3,1)),axis=1)),zeros),axis=0))

    # T is the POE transform matrix
    T = np.matmul(s[0],np.matmul(s[1],np.matmul(s[2],np.matmul(s[3],M))))
    # print T
    # original position in tool frame ([0,0,0] ---homogeneous---> [0,0,0,1])
    original_pos = np.array([0,0,0,1]);
    # new_pos is the position in coordinate system centered at the bottom center of the base
    new_pos = np.matmul(T,original_pos)
    # world_pos is the position in the world coordinate system
    rotation = [[cos(pi/2), -sin(pi/2), 0, 0],[sin(pi/2), cos(pi/2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    new_pos = np.matmul(rotation,new_pos)
    world_pos = [new_pos.item(0)+x_off, new_pos.item(1)+y_off, new_pos.item(2), new_pos.item(3)]

    phi = np.pi/2.0 - joint_angles[1] - joint_angles[2] - joint_angles[3];
    return [world_pos[0], world_pos[1], world_pos[2], phi];

def IK(pose):
    # ik parameters
    d1 = 118
    a2 = 99
    a3 = 99
    a4 = 143.6
    #world coordinate and phi
    Xw,pose=np.split(pose,[1])
    Yw,pose=np.split(pose,[1])
    Ze,phi=np.split(pose,[1])    

    
    Ye = -(float(Xw)-x_off)
    Xe = float(Yw)-y_off
    Re = (Xe**2 + Ye**2)**0.5
    Ze = float(Ze) - d1
    phi = float(phi)
    #calculating th1
    th1 = atan2(Ye,Xe)                                                      #theta 1 

    if ((Re**2+Ze**2)**0.5)>341.6:
        print 'Position too far'
        return [0,0,0,0]

    #parameters:
    

    dR = Re - a4*cos(-phi)
    dR = Re - a4*cos(phi)
    dZ = Ze + a4*sin(-phi)
    #incrementing phi to reach possible pose
    #calculating th3
    while True:
        try:
            if phi <= 0: 
                th3 = -1*acos(round((dZ**2+dR**2-a2**2-a3**2)/(2*a2*a3),6))    #theta 3
                break
            else:
                print 'position impossible to reach'
                return[0,0,0,0]
                break
            
        except ValueError:
            phi += pi/180
            print 'phi too stiff, modyfied to',phi*180/pi,'degree'
            dR = Re - a4*cos(-phi)
            dR = Re - a4*cos(phi)
            dZ = Ze + a4*sin(-phi)
            
    # return if location not valid
    print 'Re:',Re
    if (Re>341.6 or Ze>118+341.6):
        print 'error: location out of range'
        return[0,0,0,0]
    elif Re>a2+a3+a4*cos(-phi):
        print 'error: euler angle too stiff'
        return[0,0,0,0]

    #calculating th2,4
    beta = atan2(dZ,dR)
    psi = atan2(a3*sin(-th3),a2+a3*cos(-th3))
    th2 = beta+psi                                                      #theta 2
    th4 = phi - th2 - th3                                                  #theta 4

    
    #print 'IK result:',[th1,th2,th3,th4]
    Xs = [0,a2*cos(th2),a2*cos(th2)+a3*cos(th2+th3),a2*cos(th2)+a3*cos(th2+th3)+a4*cos(phi)]
    Ys = [0,a2*sin(th2),a2*sin(th2)+a3*sin(th2+th3),a2*sin(th2)+a3*sin(th2+th3)+a4*sin(phi)]
    # plt.plot(Xs,Ys,'-o')
    # plt.title('plot for X Y Z phi:' )
    # plt.show()
    th2r = pi/2 - th2
    th3r = -th3
    th4r = -th4
    return[th1,th2r,th3r,th4r]
    # phi = -th4r + (pi/2 - th2r) - th3r
    
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
# grayThreshold
def next_phi(joints):
    return 0-joints[3] + pi/2.0 - joints[1] - joints[2]


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix

    """
    a13 = T[0,2]
    a23 = T[1,2]
    a33 = T[2,2]
    a31 = T[2,0]
    a32 = T[2,1]
    #print 'a13-a31',a13,a23,a33,a32,a31
    theta = round(atan2((1-a33**2)**0.5,a33),3)
    psi = round(atan2(a13,-a23),3)
    delta = round(pi - atan2(a31,a32),3)
    euler = [psi,theta,delta]
    #print 'euler angle Z(psi)X(theta)Z(delta):',euler
    return euler
    

def get_pose_from_T(T):
    X=round(T[0,3],3)
    Y=round(T[1,3],3)
    Z=round(T[2,3],3)
    phi = round(get_euler_angles_from_T(T)[2],3)
    pose = [Y,603.25-X,Z,phi]
    #print 'pose(X Y Z phi):',pose
    return pose
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

#test code
# get_euler_angles_from_T(FK_dh(IK([304,292+100,100,-pi/2]),4))
print get_pose_from_T(FK_dh([pi/2,pi/4,0,0],4))
# print IK([x_off+200,y_off-200,118,-pi/4]);


