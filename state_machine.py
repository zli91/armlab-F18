import time
import numpy as np
from rexarm import Rexarm as rexarm
from trajectory_planner import TrajectoryPlanner as tp
from kinect import Kinect as kinect
from kinematics import *
import cv2
import freenect


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
            if(self.next_state == "blockDetectionStart"):
                self.blockDetectionStart()
            if(self.next_state == "blockMessage"):
                self.blockMessage()
            if(self.next_state == "blockDetectionEnd"):
                self.blockDetectionEnd()
            if(self.next_state == "teachNRepeat"):
                self.teachNRepeat()
            if(self.next_state == "blockDetection"):
                self.blockDetection()
            if (self.next_state == "clickNGrab"):
                self.clickNGrab()
            if (self.next_state == "pickNPlace"):
                self.pickNPlace()
            if (self.next_state == "pickNStack"):
                self.pickNStack()
            if (self.next_state == "lineUp"):
                self.lineUp()
            if (self.next_state == "stackHigh"):
                self.stackHigh()
            if (self.next_state == "buildPyramid"):
                self.buildPyramid()

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

        if(self.current_state == "blockDetectionStart"):
            if(self.next_state == "blockDetectionStart"):
                self.blockDetectionStart()
            if(self.next_state == "blockMessage"):
                self.blockMessage()
            if(self.next_state == "blockDetectionEnd"):
                self.blockDetectionEnd()

        if(self.current_state == "blockMessage"):
            if(self.next_state == "blockDetectionStart"):
                self.blockDetectionStart()
            else:
                self.idle()

        if(self.current_state == "blockDetectionEnd"):
            if(self.next_state == "idle"):
                self.idle()

        # if(self.current_state == "blockDetectionStart"):
        # if(self.current_state == "blockDetectionEnd"):
        #     self.idle();

        if (self.current_state == "teachNRepeat"):
            if(self.next_state == "teachNRepeat"):
                self.teachNRepeat()
            if(self.next_state == "recordWaypoint"):
                self.recordWaypoint()
            if(self.next_state == "play"):
                self.play()
            else:
            # if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "clickNGrab"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "pickNPlace"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "pickNStack"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "lineUp"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "stackHigh"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "buildPyramid"):
            if(self.next_state == "idle"):
                self.idle()
            if (self.next_state == "buildPyramid"):
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
        # self.rexarm.set_torque_limits([0.0]*self.rexarm.num_joints)
        self.rexarm.get_feedback()
        # self.rexarm.open_gripper()
        # cur = time.time()
        # gripper = True
        # while (True):
        #     if (time.time() - cur > 5):
        #         cur = time.time()
        #         if (gripper):
        #             self.rexarm.close_gripper()
        #             gripper = False
        #         else:
        #             self.rexarm.open_gripper()
        #             gripper = True
                

        

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()

    def execute(self):
        self.status_message = "State: Execute "
        self.current_state = "execute"
        self.next_state = "idle"      
        self.tp.wp = [[0.0, 0.0, 0.0, 0.0,0,0],
                        [1.2, 1.0, 0.9, 0.7,0,0]]
                       # [1.0, 0.8, 1.0, 1.0], # test: 1.2, 1.0, 0.9, 0.7
                       # [-1.0, -0.8, -1.0, -1.0],
                       # [-1.0, 0.8, 1.0, 1.0],
                       # [1.0, -0.8,-1.0,-1.0],
                       # [0.0, 0.0, 0.0, 0.0]]
        self.tp.execute_without_path_smoothing()
    
    # teachNRepeat state does not end until button click
    def teachNRepeat(self):
        # set the torque to 0
        self.current_state = "teachNRepeat"
        self.status_message = "State: Teach n' Repeat - torque set to 0, click \"Record Waypoint\" or \"play\""
        self.rexarm.set_torque_limits([0.0]*self.rexarm.num_joints)
        # next state is either recordWaypoint or play depending on button click

    # recordWaypoint returns to teachNRepeat
    def recordWaypoint(self):
        self.status_message = "State: Record Waypoint"
        self.next_state = "teachNRepeat"
        self.tp.set_wp()

    # play exits teachNRepeat
    def play(self):
        self.status_message = "State: Play - going to the waypoints in collect order"
        self.next_state = "idle"
        self.rexarm.set_torque_limits([0.5]*self.rexarm.num_joints)
        self.rexarm.pause(1)
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
        self.kinect.convert_to_cam = np.linalg.inv(self.kinect.convert_to_world)

        matrix_in = self.kinect.loadCameraCalibration()
        self.kinect.kinectCalibrated = True
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

    def blockDetectionStart(self):
        self.current_state = "blockDetectionStart"
        self.status_message = "State: Block Detection - Start"
        self.next_state = "blockDetectionStart"
        # self.kinect.processVideoFrame()
        # self.kinect.detectBlocksInDepthImage()
        # self.kinect.blockDetector()
        self.kinect.blockDetected = True

    def blockMessage(self):
        self.current_state = "blockMessage"
        self.status_message = "State: Block Detection - Message"
        self.next_state = "blockDetectionStart"
        self.kinect.blockMessage = True
        time.sleep(0.01)
        self.kinect.blockMessage = False


    def blockDetectionEnd(self):
        self.current_state = "blockDetectionEnd"
        self.status_message = "State: Block Detection - End"
        self.next_state = "idle"
        self.kinect.blockDetected = False

    # where you can click on a block in the video and the arm will move to grasping location, 
    # then a second click will tell the arm to move to a drop off location. 
    def clickNGrab(self):
        self.current_state = "clickNGrab"
        self.next_state = "idle"
        self.kinect.new_click = False;
        pos_list = []

        # wait for mouse click
        while (self.kinect.new_click==False): 
            self.status_message = "State: Click n' Grab - waiting for the first mouse click"
        phi = -np.pi/2
        x = self.kinect.last_click[0]
        y = self.kinect.last_click[1]
        self.kinect.new_click = False;
        
        # check if cv is calibrated

        # calculate the coordinates
        # mouse_coor = [x,y,1]
        # world_coord = np.matmul(self.kinect.convert_to_world, mouse_coor)
        # # 2 accounts for the offset from center of block to the surface
        # world_Z = self.kinect.worldHeight - 0.1236 * 1000 * np.tan(z/2842.5 + 1.1863) - 2 
        cur_pos = self.rexarm.get_positions()[:]
        world_coord = self.kinect.world_coord(x,y)
        print "click", x,y
        position = IK([world_coord[0], world_coord[1], world_coord[2]-20, phi])
        print "click", world_coord
        position_rot = [position[0], cur_pos[1], cur_pos[2], cur_pos[3]]
        position_top = IK([world_coord[0], world_coord[1], 200, phi])

        # add the waypoints into tp
        pos_list.append(position_rot)
        # pos_list.append(position_top)
        pos_list.append(position)

        for i in pos_list:
            self.tp.add_wp(i)
        finish = self.tp.execute_plan_and_grab()
        phi = next_phi(position)

        while (finish == False):
            continue
        # clear the previous positions and append the top position to the list
        del pos_list[:]
        pos_list.append(position_rot[:])

        # wait for mouse click
        while (self.kinect.new_click==False):
            self.status_message = "State: Click and Grab - waiting for the second mouse click"
        x = self.kinect.last_click[0]
        y = self.kinect.last_click[1]
        self.kinect.new_click=False
        
        # calculate the coordinates
        # z = self.kinect.currentDepthFrame[y][x]
        # mouse_coor = [x,y,1]
        # world_coord = np.matmul(self.kinect.convert_to_world, mouse_coor)
        # world_Z = self.kinect.worldHeight - 0.1236 * 1000 * np.tan(z/2842.5 + 1.1863)
        world_coord = self.kinect.world_coord(x,y);
        position = IK([world_coord[0], world_coord[1], world_coord[2]+20, phi])[:]
        position_rot = [position[0], cur_pos[1], cur_pos[2], cur_pos[3]]
        # position_top = IK([world_coord[0], world_coord[1], 200, phi])

        # add the waypoints into tp
        pos_list.append(position_rot[:])
        pos_list.append(position)
        for i in pos_list:
            self.tp.add_wp(i)
        self.tp.execute_plan_and_place()

        self.tp.add_wp(position_rot[:])
        self.tp.add_wp([0,0,0,0])
        self.tp.execute_plan()

    def pickNPlace(self):
        self.current_state = "pickNPlace"
        self.next_state = "idle"
        self.status_message = "State: Pick n' Place"
        x_off = 304  # distances from center of the bottom of ReArm to world origin
        y_off = 301.5
        phi = -np.pi/2
        positions = self.kinect.blockDetector(174,177)[:]
        input_positions = []
        for i in range(len(positions)):
            print "camera position detected"
            print positions[i]
            # grab the block
            cur_pos = self.rexarm.get_positions()[:]
            x = positions[i][0]
            y = positions[i][1]

            print "X", x
            print "y", y
            world_coord = self.kinect.world_coord(x,y)[:]
            print "world coord converted"
            print world_coord

            joints = IK([world_coord[0], world_coord[1], world_coord[2]-13, phi])[:]
            
            joints_rot = [joints[0], cur_pos[1], cur_pos[2], cur_pos[3]]
            input_positions.append(joints_rot[:])
            input_positions.append(joints[:])

            # place the block
            world_coord_p = [x_off-(world_coord[0] - x_off), world_coord[1], 40, phi]
            joints_p = IK([world_coord_p[0], world_coord_p[1], world_coord_p[2]-15, phi])[:]
            joints_rot = [joints_p[0], cur_pos[1], cur_pos[2], cur_pos[3]]
            input_positions.append(joints_rot[:])
            input_positions.append(joints_p[:])
        print "input:"
        print input_positions
        # self.rexarm.pause(3)
        self.tp.pickNPlace(input_positions)
        # self.tp.pickNPlace()

    def pickNStack(self):
        self.current_state = "pickNStack"
        self.next_state = "idle"
        self.status_message = "State: Pick n' Stack"
        x_off = 304  # distances from center of the bottom of ReArm to world origin
        y_off = 301.5
        phi = -np.pi/2
        des_pos_x = 170
        des_pos_y = 170
        des_pos_z = 23
        positions = self.kinect.blockDetector(174,177)[:]
        input_positions = []
        for i in range(len(positions)):
            phi = -np.pi/2
            print "camera position detected"
            print positions[i]
            # grab the block
            cur_pos = [0,0,0,0]
            x = positions[i][0]
            y = positions[i][1]

            world_coord = self.kinect.world_coord(x,y)

            joints = IK([world_coord[0], world_coord[1], world_coord[2]-17, phi])
            
            joints_rot = [joints[0], cur_pos[1], cur_pos[2], cur_pos[3]]
            input_positions.append(joints_rot[:])
            input_positions.append(joints[:])

            # place the block
            phi = next_phi(joints)
            # world_coord_p = [des_pos_x, des_pos_y, des_pos_z, phi]
            joints_p = IK([des_pos_x, des_pos_y, des_pos_z, phi])
            joints_rot = [joints_p[0], cur_pos[1], cur_pos[2], cur_pos[3]]
            input_positions.append(joints_rot[:])
            input_positions.append(joints_p[:])
            des_pos_z = des_pos_z+40
        print "input:"
        print input_positions
        self.tp.pickNPlace(input_positions)
        # self.tp.pickNPlace()

    def lineUp(self):
        self.current_state = "lineUp"
        self.next_state = "idle"
        self.status_message = "State: Line 'em UP "
        self.kinect.new_click = False;
        all_colors = ["black", "red", "orange", "yellow", "green", "blue", "purple", "pink"]
        destination_x = {}
        set_up = False
        # wait for mouse click
        while (set_up==False):
            while (self.kinect.new_click==False): 
                continue;
            [start_x, des_pos_y, des_pos_z] = self.kinect.world_coord(self.kinect.last_click[0], self.kinect.last_click[1])
            self.kinect.new_click = False;
            destination_x.clear()
            # Black Red Orange Yellow Green Blue Violet Pink
            for i in range(8):
                if (start_x > 550):
                    print "ERROR: start location invalid for line up"
                destination_x[all_colors[i]]=start_x
                start_x = start_x+60
            set_up = True
        print "x coord: "
        print destination_x
        self.rexarm.pause(0.5)

        self.tp.lineUpMain(self.kinect, destination_x, des_pos_y)
        # self.kinect.new_click = False;
        # all_colors = ["black", "red", "orange", "yellow", "green", "blue", "purple", "pink"]
        # destination_x = {}
        # set_up = False
        # # wait for mouse click
        # while (set_up==False):
        #     while (self.kinect.new_click==False): 
        #         self.status_message = "State: Line 'em Up - waiting for mouse click for starting location"
        #     phi = -np.pi/2
        #     [start_x, des_pos_y, des_pos_z] = self.kinect.world_coord(self.kinect.last_click[0], self.kinect.last_click[1])
        #     self.kinect.new_click = False;
        #     destination_x.clear()
        #     # Black Red Orange Yellow Green Blue Violet Pink
        #     for i in range(8):
        #         if (start_x > 550):
        #             print "ERROR: start location invalid for line up"
        #         destination_x[all_colors[i]]=start_x
        #         start_x = start_x+50
        #     set_up = True
        # print "x coord: "
        # print destination_x
        # self.status_message = "set up complete "
        # self.rexarm.pause(1)
        # des_pos_x = 0
        # phi = -np.pi/2
        # des_pos_z = 25
        # # depth ranges for layer 3, 2, 1
        # depthRange = [[160,169],[170,173],[174,177]]
        # # depthRange = [[174,177]] 
        # # positions input into tp
        # input_positions = []
        
        # for j in range(len(depthRange)):
        #     depthMin = depthRange[j][0]
        #     depthMax = depthRange[j][1]
        #     positions = self.kinect.blockDetector(depthMin,depthMax)[:]
        #     # print "positions:"
        #     # print positions
        #     # print self.kinect.detectedCubeColor
        #     for i in range(len(positions)):
        #         # print "camera position detected"
        #         # print positions[i]
        #         # grab the block

        #         # block location
        #         x = positions[i][0]
        #         y = positions[i][1]
        #         world_coord = self.kinect.world_coord(x,y)
        #         joints = IK([world_coord[0], world_coord[1], 40*j-13, phi])
        #         input_positions.append(joints[:])

        #         # x coordinate to place the block
        #         des_pos_x = destination_x[self.kinect.detectedCubeColor[i]]
        #         # place location
        #         phi = next_phi(joints)
        #         # world_coord_p = self.kinect.world_coord(des_pos_x,des_pos_y)
        #         joints_p = IK([des_pos_x, des_pos_y, 27, phi])
        #         input_positions.append(joints_p[:])

        # print input_positions
        # self.tp.lineUp(input_positions)

    def stackHigh(self):
        self.current_state = "stackHigh"
        self.next_state = "idle"
        self.status_message = "State: Stack 'em High"

        self.kinect.new_click = False;
        all_colors = ["black", "red", "orange", "yellow", "green", "blue", "purple", "pink"]

        # wait for mouse click
        while (self.kinect.new_click==False): 
            self.status_message = "State: Stack 'em High - waiting for mouse click for starting location"
        phi = -np.pi/2
        des_pos_x = self.kinect.last_click[0]
        des_pos_y = self.kinect.last_click[1]
        self.kinect.new_click = False;

        self.tp.StackHighMain(self.kinect, des_pos_x, des_pos_y)


    def buildPyramid(self):
        self.current_state = "buildPyramid"
        self.next_state = "idle"
        self.status_message = "State: Pyramid Builder"
        self.rexarm.pause(2)
        """
        get the starting location for pyramid
        """
        self.kinect.new_click = False;
        all_colors = ["black", "red", "orange", "yellow", "green", "blue", "purple", "pink"]

        # wait for mouse click
        while (self.kinect.new_click==False): 
            self.status_message = "State: Build Pyramid - waiting for mouse click for starting location"
        phi = -np.pi/2
        
        """
        for pyramid coordinates, current solution: hard coding in calculated results for pyramid
        pos should be a list of list of x, y, and z coordinates in forms of [[x, y, z], [x, y, z], ...]
        """
        destination_x = []
        set_up = False
        # wait for mouse click
        while (set_up==False):
            while (self.kinect.new_click==False): 
                continue;
            [start_x, des_pos_y, des_pos_z] = self.kinect.world_coord(self.kinect.last_click[0], self.kinect.last_click[1])
            self.kinect.new_click = False;
            destination_x.clear()
            # Black Red Orange Yellow Green Blue Violet Pink
            for i in range(8):
                if (start_x > 550):
                    print "ERROR: start location invalid for line up"
                destination_x.append(start_x)
                start_x = start_x+50
            set_up = True
        print "x coord: "
        print destination_x

        self.kinect.new_click = False;
        self.rexarm.pause(0.5)

        self.tp.lineUpMain(self.kinect, destination_x, des_pos_y)

        # click on the build Pyramid button twice to stop building pyramid
        """
        depthMin and depthMax are two constants
        """
        count = 0 # counts which position in pos list to use 
        while (self.next_state != "buildPyramid"):
            time.sleep(0.03)
            positions = self.kinect.blockDetector(174,177)[:]
            if (positions.size()!=0):
                for i in range(len(positions)):
                    x = positions[i][0]
                    y = positions[i][1]
                    world_coord = self.kinect.world_coord(x,y)
                    joints = IK([world_coord[0], world_coord[1], world_coord[2]-20, phi])
                    input_positions.append(joints[:])

                    # place location
                    phi = next_phi(joints)
                    joints_p = IK([pos[count][0], pos[count][1], pos[count][2], phi])
                    input_positions.append(joints_p[:])
                    count += 1
                self.tp.lineUp(input_positions)


    # helper function to find a empty place
    def next_loc(self, x, y):
        diff = 55
        x_off = 304  # distances from center of the bottom of ReArm to world origin
        y_off = 301.5
        x_sign = 0
        y_sign = 0
        # the block is at the right half of the plane
        if (x-x_off>0):
            x_sign = -1
        # block at left half of the plane
        else:
            x_sign = 1
        # the block is at the top half of the plane
        if (y-y_off>0):
            x_sign = -1
        # block at bottom half of the plane
        else:
            x_sign = 1
        # find a close empty position to place the block
        while (offset<200):
            if (self.kinect.depthOf(x+x_sign*diff,y)<10):
                return [x+x_sign*50,y]
            elif (self.kinect.depthOf(x,y+y_sign*diff)<10):
                return [x,y+y_sign*diff]
            elif (self.kinect.depthOf(x+x_sign*diff/2,y+y_sign*diff/2)<10):
                return [x+x_sign*diff/2,y+y_sign*diff/2]
            diff += 20
        if (self.count%3 == 0): 
            self.count += 1
            return [300, 200] #default value
        elif (self.count%3 == 1): 
            self.count += 1
            return [400, 400] #default value
        elif (self.count%3 == 2):
            self.count += 1
            return [100, 400]
