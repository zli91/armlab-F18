import time
import numpy as np
from rexarm import Rexarm as rexarm
<<<<<<< HEAD
from trajectory_planner import TrajectoryPlanner as tp
import csv
=======
>>>>>>> c9443017febd24041004f2a39105cf4cb30bc95e

"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"


    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "execute"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()  

        if(self.current_state == "recordWaypoint"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "play"):
            if(self.next_state == "idle"):
                self.idle()
                
               

    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()

    def execute(self):
        self.status_message = "State: Execute "
        self.current_state = "execute"
        self.next_state = "idle"      
        joints = [[0.0, 0.0, 0.0, 0.0],
                       [1.0, 0.8, 1.0, 1.0],
                       [-1.0, -0.8, -1.0, -1.0],
                       [-1.0, 0.8, 1.0, 1.0],
                       [1.0, -0.8,-1.0,-1.0],
                       [0.0, 0.0, 0.0, 0.0]]
        for i in range(6):
            self.rexarm.set_positions(joints[i])
            self.rexarm.pause(2)
            

    def recordWaypoint(self):
        self.status_message = "State: Record Waypoint"
        self.current_state = "recordWaypoint"
        self.next_state = "idle"
        self.tp.go()
        for i in range(len(self.tp.go()))
            self.tp.set_wp()

    def play(self):
        self.status_message = "State: Play"
        self.current_state = "play"
        self.next_state = "dile"
        self.tp.go()
        
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        print self.kinect.rgb_click_points
        print self.kinect.depth_click_points

        """TODO Perform camera calibration here"""

        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)