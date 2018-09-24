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
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
        self.wp = [[0.0, 0.0, 0.0, 0.0]];
        self.T = 0;
        self.time_factor = 20 # determines the total time motor takes from one point to the other
    
    def set_initial_wp(self):
        pass

    def set_final_wp(self, waypoint):
        pass

    def set_wp(self):
        temp = self.rexarm.get_positions()
        self.wp.append(temp[:])
        print(self.wp);

    def go(self, initial_wp, final_wp, max_speed = 2.5):
        qt0 = self.initial_wp
        qtf = self.final_wp
        print ("calculating time needed")
        self.calc_time_from_waypoints(self.initial_wp, self.final_wp)
        print("calculating cubic spline")
        cubic_coeffs = self.generate_cubic_spline(self.initial_wp, self.final_wp, self.T)
        print("moving")
        # initialize speed vector and pos_ition
        vt = [0.00001, 0.000001, 0.000001, 0.000001]
        self.rexarm.set_speeds(vt)
        current_pos = [0.0, 0.0, 0.0, 0.0]
        num_intervals = int(self.T/0.05);
        if (num_intervals < 4):
            num_intervals = 4
        time_interval = self.T/num_intervals;
        time_begin = float(time.time())
        self.rexarm.set_positions(final_wp)

        # resultFile = open("with_path_smoothing","wb")
        # writeResult = csv.writer(resultFile, delimiter=',')
        for j in range(num_intervals-1):
            # self.rexarm.set_positions(current_pos)
            # self.rexarm.pause(time_interval-0.01)
            cur_time = time_interval*(j+1)
            # temp = false;
            for k in range(len(qt0)):
                vt[k] = cubic_coeffs[k][1] + 2*cubic_coeffs[k][2]*cur_time + 3*cubic_coeffs[k][3]*cur_time*cur_time
                # current_pos[k] = cubic_coeffs[k][0] + cubic_coeffs[k][1]*cur_time + cubic_coeffs[k][2]*cur_time*cur_time + cubic_coeffs[k][3]*cur_time*cur_time*cur_time
            self.rexarm.set_speeds(vt)
            current_pos = self.rexarm.get_positions()
            # writeResult.writerow(current_pos)
            self.rexarm.pause(time_interval)
        self.rexarm.pause(time_interval)
        # self.rexarm.set_positions(final_wp)
        # resultFile.close()


    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed=2.5):
        max_velocity = [6.17, 6.17, 6.17, 12.2595];
        time = 0.0;
        print "inital: ", self.initial_wp
        print "final: ", self.final_wp
        for i in range(len(self.initial_wp)):
            qf = self.final_wp[i]
            q0 = self.initial_wp[i]
            time = max(time, abs((qf-q0)*self.time_factor/max_velocity[i]))
            print "time: ", time
        self.T = time

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        coeffs = [];
        for i in range(len(initial_wp)):
            temp = [];
            time = float(T)
            temp.append(initial_wp[i])  # a0
            temp.append(0)              # a1
            # solve for a2 and a3
            a = np.array([[time*time, time*time*time], [2*time, 3*time*time]])
            b = np.array([final_wp[i] - initial_wp[i], 0])
            x = np.linalg.solve(a,b)
            temp.append(x[0])
            temp.append(x[1])
            # temp = [a0, a1, a2, a3]
            coeffs.append(temp[:])
        print coeffs
        return coeffs


    def execute_plan(self, look_ahead=8):
        # print([max_speed]*len(self.rexarm.joints))
        # self.rexarm.set_speeds(self, [max_speed]*4)
        if len(self.wp)!= 0:
            self.rexarm.set_positions([0, 0, 0, 0])
            for i in range(len(self.wp)-1):
                self.initial_wp = self.wp[i];
                self.final_wp = self.wp[i+1];
                self.go(self.initial_wp, self.final_wp);

            with open("data.csv", 'wb') as resultFile:
                writeResult = csv.writer(resultFile, delimiter=',')
                for i in range(len(self.wp)):
                    writeResult.writerow(self.wp[i])

            self.wp = [[0.0, 0.0, 0.0, 0.0]]

    def execute_without_path_smoothing(self):
        time_begin = float(time.time())
        interval_begin = time_begin
        # with open("execute_without_path_smoothing.csv", 'wb') as resultFile:
        #     writeResult = csv.writer(resultFile, delimiter=',')
        for i in range(len(self.wp)):
            self.rexarm.set_positions(self.wp[i])
                # while (time.time() - time_begin < 2):
                #     if (time.time() - interval_begin >= 0.05):
                #         interval_begin = time.time()
                #         current_pos = self.rexarm.get_positions()
                #         writeResult.writerow(current_pos)
            self.rexarm.pause(2)
        self.wp = [[0.0, 0.0, 0.0, 0.0]]