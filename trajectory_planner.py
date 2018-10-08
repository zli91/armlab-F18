import numpy as np 
import time
from rexarm import Rexarm as rexarm
from kinect import Kinect as kinect
import csv
import math
from kinematics import *

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.kinect = kinect
        self.num_joints = 4
        self.initial_wp = [0.0]*4
        self.final_wp = [0.0]*4
        self.dt = 0.05 # command rate
        self.wp = [];
        self.T = 0;
        self.time_factor = 6 # determines the total time motor takes from one point to the other
        self.look_ahead = 8 # determines how much time to look ahead when planning

    def set_initial_wp(self):
        self.initial_wp = self.rexarm.get_positions()[0:4]

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint;

    def add_wp(self, joints):
        self.wp.append(joints[:])
        print(self.wp)

    def set_wp(self):
        temp = self.rexarm.get_positions()[0:4]
        self.wp.append(temp)
        print(self.wp);

    def open_gripper(self):
        self.rexarm.open_gripper()

    def close_gripper(self):
        self.rexarm.close_gripper()

    def go(self, max_speed = 2.5):
        qt0 = self.initial_wp[:]
        qtf = self.final_wp[:]

        # add gripper positions to pos
        pos = self.final_wp[:]

        # how gripper is toggled
        pos.append(self.rexarm.t_gripper)

        # if the gripper is open
        if (self.rexarm.gripper_open == True): # gripper open
            pos.append(1.4)
        else:
            pos.append(0)

        # print "final_wp:", qtf
        # print ("calculating time needed")
        self.calc_time_from_waypoints(self.initial_wp, self.final_wp)
        # self.T = float(2)
        if (self.T<0.01):
            self.rexarm.set_positions(pos)
            return

        # print("calculating cubic spline")
        coeffs = self.generate_cubic_spline(self.initial_wp, self.final_wp, self.T)[:]
        # print("moving")
        vt = [0.00001,0.00001,0.000001,0.00001]


        num_intervals = int(self.T/0.08);
        if (num_intervals < 4):
            num_intervals = 4
        time_interval = float(self.T/num_intervals);
        # while (time_interval*1000 < look_ahead):
        #     look_ahead /= 2;

        # initialize speed vector and position
        for k in range(len(qt0)):
            vt[k] = (coeffs[k][1] + 2*coeffs[k][2]*self.look_ahead/1000 + 3*coeffs[k][3]*self.look_ahead/1000*self.look_ahead/1000)
            # current_pos[k] = coeffs[k][0] + coeffs[k][1]*cur_time + coeffs[k][2]*cur_time*cur_time + coeffs[k][3]*cur_time*cur_time*cur_time
        vt.append(20)
        vt.append(20)
        self.rexarm.set_speeds(vt)

        # start moving
        resultFile = open("with_path_smoothing.csv","wb")
        # resultFileVel = open("vel_with_path_smoothing.csv","wb")
        writeResult = csv.writer(resultFile, delimiter=',')
        # writeResultVel = csv.writer(resultFileVel, delimiter=',')
        
        time_begin = time.time();

        self.rexarm.set_positions(pos)
        self.rexarm.pause(time_interval-self.look_ahead/1000)
        # while (time.time()-time_begin<self.T):
        for j in range(num_intervals-1):
            # self.rexarm.set_positions(current_pos)
            # self.rexarm.pause(time_interval-0.01)
            cur_time = time_interval*(j+1)
            # cur_time = time.time() - time_begin;
            for k in range(len(qt0)):
                vt[k] = (coeffs[k][1] + 2*coeffs[k][2]*cur_time + 3*coeffs[k][3]*cur_time*cur_time)/1.8
                # current_pos[k] = coeffs[k][0] + coeffs[k][1]*cur_time + coeffs[k][2]*cur_time*cur_time + coeffs[k][3]*cur_time*cur_time*cur_time
            vt.append(20)
            vt.append(20)
            self.rexarm.set_speeds(vt)
            # time.sleep(0.05)
            self.rexarm.pause(time_interval)

            # self.rexarm.set_positions(current_pos)
            write_pos = self.rexarm.get_positions()[:]
            # write_pos+=current_pos[:]
            write_pos.append(time.time()-time_begin)
            writeResult.writerow(write_pos)
        self.rexarm.pause(time_interval)
        # self.rexarm.set_positions(pos)
        resultFile.close()
        print "complete moving"
        return True


    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed=2.5):
        max_velocity = [12.2595, 12.2595, 12.2595, 11.89]; #[MX, MX, MX, AX]
        time = float(0.0);
        # print "inital: ", self.initial_wp
        # print "final: ", self.final_wp
        for i in range(len(self.initial_wp)):
            qf = self.final_wp[i]
            q0 = self.initial_wp[i]
            time = max(time, abs((qf-q0)*self.time_factor/max_velocity[i]))
            # print "time: ", time
        self.T = float(time)

    # used in go(). returns the cooefficients in cubic spline
    def generate_cubic_spline(self, initial_wp, final_wp, T):
        coeffs = [];
        cubic_matrix = [[1,0,0,0],[0,1,0,0],[1,T,T*T,T*T*T], [0,1,2*T,3*T*T]]
        for i in range(len(self.initial_wp)):
            temp = [];
            conditions = [self.initial_wp[i], 0, self.final_wp[i], 0]
            temp = np.dot(np.linalg.inv(cubic_matrix),np.transpose(conditions))
            coeffs.append(temp[:])
        # print coeffs
        return coeffs

    def tp_toggle_gripper(self, orient, pose, flag):
        self.rexarm.toggle_gripper(orient, pose, flag);

    def execute_plan(self, look_ahead=8):
        # print([max_speed]*len(self.rexarm.joints))
        # self.rexarm.set_speeds(self, [max_speed]*4)
        if len(self.wp)!= 0:
            # self.initial_wp = self.wp[0];
            self.initial_wp = self.rexarm.get_positions()[0:4]
            while (len(self.wp)!=0):
                self.final_wp = self.wp.pop(0);
                self.go();
                self.initial_wp = self.rexarm.get_positions()[0:4];
            # with open("data.csv", 'wb') as resultFile:
            #     writeResult = csv.writer(resultFile, delimiter=',')
            #     for i in range(len(self.wp)):
            #         writeResult.writerow(self.wp[i])
            # self.rexarm.pause(1)
            del self.wp[:]

    def execute_without_path_smoothing(self):
        time_begin = float(time.time())
        interval_begin = time_begin
        # with open("execute_without_path_smoothing.csv", 'wb') as resultFile:
        #     writeResult = csv.writer(resultFile, delimiter=',')
        for i in range(len(self.wp)-1):
            self.rexarm.set_positions(self.wp[i+1])
                # while (time.time() - time_begin < 2):
                #     if (time.time() - interval_begin >= 0.05):
                #         interval_begin = time.time()
                #         current_pos = self.rexarm.get_positions()
                #         writeResult.writerow(current_pos)
            self.rexarm.pause(2)
        # self.wp = [[0.0, 0.0, 0.0, 0.0]]

    def execute_plan_and_grab(self, look_ahead=8):
        # print([max_speed]*len(self.rexarm.joints))
        # self.rexarm.set_speeds(self, [max_speed]*4)
        if len(self.wp)!= 0:
            finish = False
            self.open_gripper()
            # self.initial_wp = self.wp[0];
            self.initial_wp = self.rexarm.get_positions()[0:4]
            while (len(self.wp)!=0):
                self.final_wp = self.wp.pop(0);
                self.go();
                self.initial_wp = self.rexarm.get_positions()[0:4];
            self.close_gripper()
            time.sleep(0.5)

    def execute_plan_and_place(self, look_ahead=8):
        # print([max_speed]*len(self.rexarm.joints))
        # self.rexarm.set_speeds(self, [max_speed]*4)
        if len(self.wp)!= 0:
            # self.initial_wp = self.wp[0];
            self.initial_wp = self.rexarm.get_positions()[0:4]
            while (len(self.wp)!=0):
                self.final_wp = self.wp.pop(0);
                self.go();
                self.initial_wp = self.rexarm.get_positions()[0:4];
            self.open_gripper()
            time.sleep(0.5)

    def pickNPlace(self, positions):
        y_off = 301.5
        phi = -np.pi/2
        pick = True
        pre_pos = [0,0,0,0]
        first = True
        while(len(positions)>0):
            if (pick==True):
                cur_pos = positions.pop(0)[:]
                self.add_wp(pre_pos)
                self.add_wp(cur_pos)
                pre_pos = cur_pos[:]
                cur_pos = positions.pop(0)[:]
                self.add_wp(cur_pos)
                self.execute_plan_and_grab()
                # pre_pos = cur_pos[:]
                pick = False;
            else:
                self.add_wp(pre_pos)
                cur_pos = positions.pop(0)[:]
                pre_pos = cur_pos[:]
                self.add_wp(cur_pos)
                cur_pos = positions.pop(0)[:]
                self.add_wp(cur_pos)
                self.execute_plan_and_place()
                pick = True
        self.add_wp(pre_pos)
        self.add_wp([0,0,0,0])
        self.execute_plan()

    def lineUp(self, positions):
        pick = True
        pre_pos = [0,0,0,0] # start position
        cur_pos = []
        cur_pos_rot = []
        while(len(positions)>0):
            if (pick==True):
                # self.rexarm.toggle_gripper(0.0)
                cur_pos = positions.pop(0)[:]
                self.add_wp(pre_pos)
                # self.execute_plan()
                # self.rexarm.toggle_gripper(0.0)
                cur_pos_rot = [cur_pos[0], pre_pos[1], pre_pos[2], pre_pos[3]]
                self.add_wp(cur_pos_rot)
                pre_pos = cur_pos_rot[:]
                self.add_wp(cur_pos)
                self.execute_plan_and_grab()
                
                pick = False;
            else:
                # self.rexarm.toggle_gripper(np.pi/2)
                cur_pos = positions.pop(0)[:]
                cur_pos_rot = [cur_pos[0], pre_pos[1], pre_pos[2], pre_pos[3]]
                self.add_wp(pre_pos)
                # self.execute_plan()
                
                # self.rexarm.toggle_gripper(np.pi/2)
                pre_pos = cur_pos_rot[:]
                self.add_wp(cur_pos_rot)
                self.add_wp(cur_pos)
                self.execute_plan_and_place()
                
                pick = True
        self.add_wp(pre_pos)
        self.add_wp([0,0,0,0])
        self.execute_plan()

    def lineUpMain(self, kinect_ins, x_coord, des_pos_y):
        
        all_colors = ["black", "red", "orange", "yellow", "green", "blue", "purple", "pink"]
        destination_x = x_coord
        des_pos_x = 0
        phi = -np.pi/2
        des_pos_z = 25
        # depth ranges for layer 3, 2, 1
        depthRange = [[160,169],[170,173],[174,177]]
        # depthRange = [[174,177]] 
        # positions input into tp
        input_positions = []
        pre_top= [0,0,0,0]
        pre_up = [0,0,0,0]

        for j in range(len(depthRange)):
            depthMin = depthRange[j][0]
            depthMax = depthRange[j][1]
            positions = kinect_ins.blockDetector(depthMin,depthMax)[:]
            # print "positions:"
            # print positions
            # print self.kinect.detectedCubeColor
            for i in range(len(positions)):
                # x coordinate to place the block
                if (destination_x.get(kinect_ins.detectedCubeColor[i],0)!=0):
                    des_pos_x = destination_x[kinect_ins.detectedCubeColor[i]]
                    del destination_x[kinect_ins.detectedCubeColor[i]]
                else:
                    continue

                x = positions[i][0]
                y = positions[i][1]
                world_coord = kinect_ins.world_coord(x,y)
                joints = IK([world_coord[0], world_coord[1], world_coord[2]-13, phi])
                # self.rexarm.armth5([world_coord[0], world_coord[1], world_coord[2]-13, phi])
                joints_up = IK([world_coord[0], world_coord[1], world_coord[2]+40, phi])
                joints_top = [joints_up[0], 0, 0, 0]

                self.add_wp(pre_up)
                self.add_wp(pre_top)
                self.execute_plan()
                self.tp_toggle_gripper(kinect_ins.cubeOrient[i], [world_coord[0], world_coord[1], world_coord[2]+40, phi],0)
                print 'tp_world coord',world_coord
                self.add_wp(joints_top)
                self.add_wp(joints_up[:])
                # self.execute_plan()

                pre_top = joints_top
                pre_up = joints_up

                self.add_wp(joints[:])
                self.execute_plan_and_grab()
                # self.rexarm.close_gripper()
                # self.rexarm.pause(0.5)

                
                # place location
                phi = next_phi(joints)
                # world_coord_p = self.kinect.world_coord(des_pos_x,des_pos_y)
                joints_p = IK([des_pos_x, des_pos_y, 27, phi])
                joints_p_up = IK([des_pos_x, des_pos_y, 77, phi])
                joints_p_top = [joints_p_up[0], 0, 0, 0]

                self.add_wp(pre_up)
                self.add_wp(pre_top)
                self.execute_plan()
                self.tp_toggle_gripper(kinect_ins.cubeOrient[i],[des_pos_x, des_pos_y, 27, phi],1)

                self.add_wp(joints_p_top[:])
                self.add_wp(joints_p_up[:])
                # self.execute_plan()
                # self.execute_plan()
                # self.rexarm.toggle_gripper(np.pi/2)


                pre_up = joints_p_up;
                pre_top = joints_p_top;

                self.add_wp(joints_p[:])
                self.execute_plan_and_place()
                # self.rexarm.open_gripper()
                # self.rexarm.pause(0.5)

        self.add_wp(pre_up)
        self.add_wp(pre_top)
        self.add_wp([0,0,0,0])
        self.execute_plan()