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

            with open("data.csv", 'wb') as resultFile:
                writeResult = csv.writer(resultFile, delimiter=',')
                for i in range(len(self.wp)):
                    writeResult.writerow(self.wp[i])

            self.wp = []

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

    def pickNPlace(self, positions):
        # x_off = 304  # distances from center of the bottom of ReArm to world origin
        # y_off = 301.5
        # phi = -np.pi/2
        # self.kinect.blockDetector()
        # input_positions = []
        # for i in range(len(positions)):
        #     # grab the block
        #     cur_pos = self.rexarm.get_positions()[:]
        #     world_coord = self.kinect.world_coord(positions[i][0],positions[i][1]);
        #     joints = IK([world_coord[0], world_coord[1], world_coord[2]-20, phi])
            
        #     joints_rot = [joints[0], cur_pos[1], cur_pos[2], cur_pos[3]]
        #     self.wp.append(world_coord[:])
        #     self.wp.append(joints_rot[:])
        #     self.execute_plan_and_grab()

        #     # place the block
        #     self.wp.append(joints_rot[:])
        #     joints = IK([world_coord[0], world_coord[1], world_coord[2]-20, phi])
        #     world_coord_p = [world_coord[0]-x_off+world_coord[0], world_coord[1]-y_off+world_coord[1], world_coord[2]-20, phi]
        #     joints_rot = [joints[0], cur_pos[1], cur_pos[2], cur_pos[3]]
        #     self.wp.append(world_coord_p[:])
        #     self.wp.append(joints_rot[:])
        #     self.execute_plan_and_place()
        # self.tp.pickNPlace(input_positions)
        x_off = 304  # distances from center of the bottom of ReArm to world origin
        y_off = 301.5
        phi = -np.pi/2
        pick = True
        pre_pos = [0,0,0,0]
        first = True
        while(len(positions)>0):
            # if (pick==True and first==True):
            #     cur_pos = positions.pop(0)[:]
            #     self.add_wp(pre_pos)
            #     self.add_wp(cur_pos)
            #     pre_pos = cur_pos[:]
            #     cur_pos = positions.pop(0)[:]
            #     self.add_wp(cur_pos)
            #     self.execute_plan_and_grab()
            #     # pre_pos = cur_pos[:]
            #     pick = False;
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
                self.rexarm.toggle_gripper(0.0)
                cur_pos = positions.pop(0)[:]
                self.add_wp(pre_pos)
                cur_pos_rot = [cur_pos[0], pre_pos[1], pre_pos[2], pre_pos[3]]
                self.add_wp(cur_pos_rot)
                pre_pos = cur_pos_rot[:]
                self.add_wp(cur_pos)
                self.execute_plan_and_grab()
                pick = False;
            else:
                self.rexarm.toggle_gripper(np.pi/2)
                cur_pos = positions.pop(0)[:]
                cur_pos_rot = [cur_pos[0], pre_pos[1], pre_pos[2], pre_pos[3]]
                self.add_wp(cur_pos_rot)
                pre_pos = cur_pos_rot[:]
                self.add_wp(cur_pos_rot)
                self.add_wp(cur_pos)
                self.execute_plan_and_place()
                pick = True
        self.add_wp(pre_pos)
        self.add_wp([0,0,0,0])
        self.execute_plan()
