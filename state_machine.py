import time
import numpy as np
from rexarm import Rexarm as rexarm
from trajectory_planner import TrajectoryPlanner as tp
from kinect import Kinect as kinect



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
            if(self.next_state == "recordWaypoint"):
                self.recordWaypoint()
            if(self.next_state == "play"):
                self.play()
            if(self.next_state == "blockDetection"):
                self.blockDetection()
                
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

        if(self.current_state == "blockDetection"):
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
        self.tp.wp = [[0.0, 0.0, 0.0, 0.0],
                       [1.2, 1.0, 0.9, 0.7]]
                       # [-1.0, -0.8, -1.0, -1.0],
                       # [-1.0, 0.8, 1.0, 1.0],
                       # [1.0, -0.8,-1.0,-1.0],
                       # [0.0, 0.0, 0.0, 0.0]]
        self.tp.execute_without_path_smoothing()
            

    def recordWaypoint(self):
        self.status_message = "State: Record Waypoint"
        self.current_state = "recordWaypoint"
        self.next_state = "idle"
        self.tp.set_wp()

    def play(self):
        self.status_message = "State: Play"
        self.current_state = "play"
        self.next_state = "idle"
        self.tp.execute_plan()
        
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.status_message = "State: calibratie"
        self.next_state = "idle"
        # self.tp.go(max_speed=2.0)
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
        
        world_coordinates = np.array([[0,0], [0,603.25], [608, 603.25], [608,0], [304, 301.625]])
        # print self.kinect.rgb_click_points
        # print self.kinect.depth_click_points
        """TODO Perform camera calibration here"""
        # calculate the affine transformation from rgb to depth
        # use self derived: depth_click_points first
        self.kinect.depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.depth_click_points, self.kinect.rgb_click_points, len(self.kinect.depth_click_points))

        # calculate the affine transformation from rgb to world
        world_affine = self.kinect.getAffineTransform(self.kinect.rgb_click_points, world_coordinates, 4)
        self.kinect.convert_to_world = np.append(world_affine, [[0, 0, 1]], axis=0)

        matrix_in = self.kinect.loadCameraCalibration()
        self.kinect.kinectCalibrated = True
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

    def blockDetection(self):
        self.current_state = "blockDetection"
        self.status_message = "State: Block Detection"
        self.next_state = "idle"
        self.kinect.detectBlocksInDepthImage()
        self.kinect.blockDetector()
