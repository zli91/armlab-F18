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
        self.wp = [];
    
    def set_initial_wp(self):
        pass

    def set_final_wp(self, waypoint):
        pass

    def set_wp(self):
        self.wp.append(self.rexarm.get_positions())
        print(self.wp[-1]);

    def go(self, max_speed = 2.5):
        self.rexarm.set_speeds(self, [max_speed]*len(self.rexarm.joints))
        for i in range(len(self.wp)):
            self.rexarm.set_positions(self.wp[i])
        with open("data.csv", 'wb') as resultFile:
            writeResult = csv.writer(resultFile, delimiter=',')
            for i in range(len(self.wp)):
                writeResult.writerow(self.wp[i])
        del self.wp[:]


    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        pass

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        pass

    def execute_plan(self, plan, look_ahead=8):
        pass