import numpy as np 
import time
from rexarm import Rexarm as rexarm
import csv

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
    
    def set_initial_wp(self):
        pass

    def set_final_wp(self, waypoint):
        pass

    def set_wp(self):
        temp = self.rexarm.get_positions()
        self.wp.append(temp[:])
        print(self.wp);

    def go(self, max_speed = 2.5):
        # print([max_speed]*len(self.rexarm.joints))
        # self.rexarm.set_speeds(self, [max_speed]*4)
        v_t0 = 0
        v_tf = 0
        self.rexarm.set_positions(self.wp[0])
        for i in range(len(self.wp)-1):
            qt0 = self.wp[i];
            qtf = self.wp[i+1];
            self.initial_wp = self.wp[i];
            self.final_wp = self.wp[i+1];
            # self.rexarm.set_positions(self.wp[i+1])
            
            # generate the cublic spline
            self.calc_time_from_waypoints(self.initial_wp, self.final_wp)
            cubic_coeffs = self.generate_cubic_spline(self.initial_wp, self.final_wp, self.T)
            print("moving")
            # initialize speed vector and position
            vt = [0.0, 0.0, 0.0, 0.0]
            self.rexarm.set_speeds(vt)
            current_pos = [0.0, 0.0, 0.0, 0.0]
            num_intervals = 20;
            time_interval = self.T/num_intervals;
            time_begin = float(time.time())

            for j in range(num_intervals):
                cur_time = time_interval*(j+1)
                # temp = false;
                for k in range(len(qt0)):
                    vt[k] = cubic_coeffs[k][1] + 2*cubic_coeffs[k][2]*cur_time + 3*cubic_coeffs[k][3]*cur_time*cur_time
                    current_pos[k] = cubic_coeffs[k][0] + cubic_coeffs[k][1]*cur_time + cubic_coeffs[k][2]*cur_time*cur_time + cubic_coeffs[k][3]*cur_time*cur_time*cur_time
                self.rexarm.set_speeds(vt)
                self.rexarm.set_positions(current_pos)
                self.rexarm.pause(time_interval-0.02)
                # while (float(time.time())-time_begin < cur_time+time_interval):
                #     continue
                print(vt)

        with open("data.csv", 'wb') as resultFile:
            writeResult = csv.writer(resultFile, delimiter=',')
            for i in range(len(self.wp)):
                writeResult.writerow(self.wp[i])
        del self.wp[:]


    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed=2.5):
        # coeff = []
        # for i in range(len(initial_wp)):
        #     qf = final_wp[i]
        #     q0 = initial_wp[i]
        #     result = [-2*(qf-q0), 3*(qf-q0), q0]
        #     coeff.append(result[:])
        self.T = 2.5

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


    def execute_plan(self, plan, look_ahead=8):
        pass