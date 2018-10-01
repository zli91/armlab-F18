import numpy as np 
import time
from rexarm import Rexarm as rexarm
import csv
import math

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = 4
        self.initial_wp = [0.0]*4
        self.final_wp = [0.0]*4
        self.dt = 0.05 # command rate
        self.wp = [];
        self.T = 0;
        self.time_factor = 10 # determines the total time motor takes from one point to the other
        self.look_ahead = self.time_factor # determines how much time to look ahead when planning

    def set_initial_wp(self):
        self.initial_wp = self.rexarm.get_positions()[0:4]

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint;

    def add_wp(self, joints):
        self.wp.append(joint[:])
        print(self.wp)

    def set_wp(self):
        temp = self.rexarm.get_positions()[0:4]
        self.wp.append(temp)
        print(self.wp);

    def go(self, max_speed = 2.5):
        qt0 = self.initial_wp
        qtf = self.final_wp
        print ("calculating time needed")
        self.calc_time_from_waypoints(self.initial_wp, self.final_wp)
        # self.T = float(2)
        print("calculating cubic spline")
        coeffs = self.generate_cubic_spline(self.initial_wp, self.final_wp, self.T)[:]
        print("moving")
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
        vt.append(0)
        vt.append(0)
        self.rexarm.set_speeds(vt)
        # start moving
        self.rexarm.pause(self.look_ahead/1000)
        resultFile = open("with_path_smoothing.csv","wb")
        # resultFileVel = open("vel_with_path_smoothing.csv","wb")
        writeResult = csv.writer(resultFile, delimiter=',')
        # writeResultVel = csv.writer(resultFileVel, delimiter=',')
        
        time_begin = time.time();
        pos = self.final_wp
        pos.append(0)
        pos.append(0)
        self.rexarm.set_positions(pos)
        # while (time.time()-time_begin<self.T):
        for j in range(num_intervals-1):
            # self.rexarm.set_positions(current_pos)
            # self.rexarm.pause(time_interval-0.01)
            cur_time = time_interval*(j+1)
            #     # temp = false;
            # cur_time = time.time() - time_begin;
            for k in range(len(qt0)):
                print coeffs[k][2], coeffs[k][3]
                vt[k] = (coeffs[k][1] + 2*coeffs[k][2]*cur_time + 3*coeffs[k][3]*cur_time*cur_time)
                # current_pos[k] = coeffs[k][0] + coeffs[k][1]*cur_time + coeffs[k][2]*cur_time*cur_time + coeffs[k][3]*cur_time*cur_time*cur_time
            vt.append(0)
            vt.append(0)
            self.rexarm.set_speeds(vt)
            # time.sleep(0.05)
            self.rexarm.pause(time_interval)
            # self.rexarm.set_positions(current_pos)
            write_pos = self.rexarm.get_positions()[:]
            write_pos.append(time.time()-time_begin)
            writeResult.writerow(write_pos)
            # write_vel = self.rexarm.get_speeds()[:]
            # write_vel.append(vt[0])
            # write_vel.append(vt[1])
            # write_vel.append(vt[2])
            # write_vel.append(vt[3])
            # write_vel.append(time.time()-time_begin)
            # writeResultVel.writerow(write_vel)
            # and time_interval
            # self.rexarm.pause(time_interval-look_ahead/1000)
        # self.rexarm.pause(time_interval)
        # self.rexarm.set_positions(final_wp)
        self.rexarm.pause(time_interval)
        resultFile.close()


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
            print "time: ", time
        self.T = float(time)

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        coeffs = [];
        print T
        cubic_matrix = [[1,0,0,0],[0,1,0,0],[1,T,T*T,T*T*T], [0,1,2*T,3*T*T]]
        print cubic_matrix
        for i in range(len(self.initial_wp)):
            temp = [];
            # time = float(T)
            conditions = [self.initial_wp[i], 0, self.final_wp[i], 0]
            # temp.append(initial_wp[i])  # a0
            # temp.append(0)              # a1
            # solve for a2 and a3
            # a = np.array([[time*time, time*time*time], [2*time, 3*time*time]])
            # b = np.array([final_wp[i] - initial_wp[i], 0])
            # x = np.linalg.solve(a,b)
            temp = np.dot(np.linalg.inv(cubic_matrix),np.transpose(conditions))
            # temp = [a0, a1, a2, a3]
            coeffs.append(temp[:])
        print coeffs
        return coeffs

    # def generate_quintic_poly(self, initial_wp, final_wp, T):
    #     quintic_array = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],[1,T,T*T,T*T*T,T*T*T*T,T*T*T*T*T],
    #         [0,1,2*T,3*T*T,4*T*T*T,5*T*T*T*T],[0,0,2,6*T,12*T*T,20*T*T*T]])

    #     coeffs = [];
    #     conditions = [0, 0, 0, ]
    #     for i in range(len(initial_wp)):
    #         temp = [];
    #         time = float(T)
    #         conditions = [initial_wp[i], 0, 0, final_wp[i], 0, 0]    # [q0 v0 a0 qf vf af]
    #         coeffs.append(np.matmul(np.linalg.inv(quintic_array), conditions)
    #         # temp = [a0, a1, a2, a3]
    #     print coeffs
    #     return coeffs 

    def execute_plan(self, look_ahead=8):
        # print([max_speed]*len(self.rexarm.joints))
        # self.rexarm.set_speeds(self, [max_speed]*4)
        if len(self.wp)!= 0:
            # self.initial_wp = self.wp[0];
            while (len(self.wp)!=0):
                self.final_wp = self.wp.pop(0);
                self.go(self.initial_wp, self.final_wp, self.look_ahead);
                self.initial_wp = self.get_positions[0:4];

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