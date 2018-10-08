import cv2
import numpy as np
from numpy.linalg import pinv
from PyQt4.QtGui import QImage
import freenect
import os
import argparse

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        self.convert_to_world = np.array([])
        self.convert_to_cam = np.array([])
        self.cubeContours = np.array([])
        self.contoursByDepth = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
        self.worldHeight = 942.0
        self.x_off = 304.88  # distances from center of the bottom of ReArm to world origin
        self.y_off = 301.5

        self.blockDetected = False

        self.blockMessage = False

        self.detectDepth = []
        self.detectedCubeColor = []
        self.cubeOrient = []
        

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
            if(self.blockDetected):
                self.processVideoFrame()
        else:
            self.loadVideoFrame()

    def world_coord(self, x, y):
        self.captureDepthFrame()
        height_y = 920.5
        height_x = 918.5
        # check if cv is calibrated
        if (self.kinectCalibrated == True):
            z = self.currentDepthFrame[y][x]
            # while (z>722):
            #     self.captureDepthFrame()
            #     z = self.currentDepthFrame[y][x]
        else:
            print("ERROR: Camera Calibrate should be completed prior to Click and Grab")
            return
        # calculate the coordinates
        mouse_coor = [x,y,1]
        # print "to world mouse coor"
        # print mouse_coor
        world_coord = np.matmul(self.convert_to_world, mouse_coor)
        # print "converted world coord: "
        # print world_coord
        # x and y coordinates converting to board without pinhole correction
        cam_X = world_coord[0]
        cam_Y = world_coord[1]
        # actual dheight above board of current point
        world_Z = self.worldHeight - 0.1236 * 1000 * np.tan(z/2842.5 + 1.1863)
        # converting x in pinhold to x in world
        d = float((self.x_off-cam_X)*(height_x-world_Z)/height_x)
        world_X = self.x_off-d;
        # converting y in pinhole to y in world
        d = float((self.y_off-cam_Y)*(height_y-world_Z)/height_y)
        world_Y = self.y_off-d
        
        # print "world coord after pin hole"
        # print [world_X, world_Y, world_Z]
        return [world_X, world_Y, world_Z]
        # return [cam_X,cam_Y,world_Z]

    def world_coord_tp(self, x, y):
        x = int(x)
        y = int(y)
        self.captureDepthFrame()
        height_y = 920.5
        height_x = 918.5
        # check if cv is calibrated
        if (self.kinectCalibrated == True):
            z = self.currentDepthFrame[y][x]
        else:
            print("ERROR: Camera Calibrate should be completed prior to Click and Grab")
            return
        print "camera z:"
        print z
        # calculate the coordinates
        mouse_coor = [x,y,1]
        print "to world mouse coor"
        print mouse_coor
        world_coord = np.matmul(self.convert_to_world, mouse_coor)
        print "converted world coord: "
        print world_coord
        # x and y coordinates converting to board without pinhole correction
        cam_X = world_coord[0]
        cam_Y = world_coord[1]
        # actual dheight above board of current point
        world_Z = self.worldHeight - 0.1236 * 1000 * np.tan(z/2842.5 + 1.1863)
        # converting x in pinhold to x in world
        # d = float((self.x_off-cam_X)*(height_x-world_Z)/height_x)
        # world_X = self.x_off-d;
        # # converting y in pinhole to y in world
        # d = float((self.y_off-cam_Y)*(height_y-world_Z)/height_y)
        # world_Y = self.y_off-d
        
        # print "world coord after pin hole"
        # print [world_X, world_Y, world_Z]
        # return [world_X, world_Y, world_Z]
        return [world_coord[0], world_coord[1], world_Z]


    def processVideoFrame(self):
        self.blockDetector(174,177)
        # draw contours
        cv2.drawContours(self.currentVideoFrame,self.rectVertex,-1,(0,255,0),3)



    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame[:]
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2, num):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        pts1 = coord1[0:num].astype(np.float32)
        pts2 = coord2[0:num].astype(np.float32)
        A = []
        At = []
        b = []
        x = []
        for i in range(num):
            A.append([pts1[i][0], pts1[i][1], 1, 0, 0, 0])
            A.append([0, 0, 0, pts1[i][0], pts1[i][1], 1])
            b.append([pts2[i][0]])
            b.append([pts2[i][1]])
        At = np.transpose(A)
        x = np.matmul(np.matmul((pinv(np.matmul(At, A))), At), b)

        result = np.array([[x.item(0), x.item(1), x.item(2)], [x.item(3), x.item(4), x.item(5)]])
        # print cv2.getAffineTransform(pts1,pts2)
        # return cv2.getAffineTransform(pts1,pts2)
        # self.depth2rgb_affine = result
        # print result
        return result


    def registerDepthFrame(self, frame):
        """
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        # np.clip(frame,0,2**10 - 1,frame)
        # frame >>= 2
        # frame = frame.astype(np.uint8)
        rows, cols = frame.shape[:2]

        # print rows, cols
        dst = cv2.warpAffine(frame, self.depth2rgb_affine, (cols, rows))
        # rows, cols = dst.shape[:2]
        # print rows, cols
        return dst

        

    def loadCameraCalibration(self):
        """
        Load camera intrinsic matrix from file.
        """
        # get the intrinsic matrix from the file
        matrix_in = np.genfromtxt(os.path.join(os.path.dirname(__file__), "util/intrinsic_matrix.csv"), delimiter=",")
        return matrix_in
    
    def blockDetector(self, boundMin, boundMax):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        self.contoursByDepth = self.detectBlocksInDepthImage(boundMin, boundMax)[:]
        cubeColor = ['black','red','orange','yellow','green','blue','purple','pink']
        # rgbBoundaries = [ # b,g,r
        #     ([4, 160, 240], [50, 210, 253]), # yellow
        #     ([10, 85, 195], [30, 130, 205]), # orange
        #     ([85, 48, 189], [110, 60, 210]), # pink
        #     ([10, 21, 21], [50, 30, 38]), # black
        #     ([26, 22, 140], [60, 38, 160]), # red
        #     ([110, 65, 120], [127, 86, 140]), # purple
        #     ([94, 120, 87],[108, 130, 105]), # greens
        #     ([130, 95, 80], [158, 99, 85]) # blue
        #     ]
        hsvBoundaries = [ # h,s,v
            ([0, 0, 30], [255, 255, 120]), # black
            ([161, 100, 135], [200, 240, 200]), # red
            ([0, 140, 200], [15, 240, 255]), # orange
            ([0, 0, 200], [45, 150, 255]), # yellow
            ([40, 80, 135],[80, 150, 170]), # green
            ([85, 100, 135], [130, 200, 230]), # blue
            ([131, 60, 122], [160, 140, 180]), # purple
            ([150, 130, 210], [180, 190, 255]), # pink
            ]

        ### color detection in rgb image
        # r = self.rgbImage[centerY][centerX][2]
        # g = self.rgbImage[centerY][centerX][1]
        # b = self.rgbImage[centerY][centerX][0]

        self.cubeCenter = []
        del self.detectedCubeColor[:]
        self.cubeContours = []
        self.rectVertex = []
        del self.cubeOrient[:]
        # vertexCoordInWorld = []
        
        camera_coord = []
        colorDetectionPoints = []

        print "debug: hsv"
        print len(hsvBoundaries)
        for j in range(len(hsvBoundaries)):
            (lower,upper) = hsvBoundaries[j]
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")        
            
            print "debug: contourDepth"
            print len(self.contoursByDepth)  
            for i in range(len(self.contoursByDepth)):      
                # find center of mass
                cubeMoment = cv2.moments(self.contoursByDepth[i])
                centerX = int(cubeMoment["m10"] / cubeMoment["m00"])
                centerY = int(cubeMoment["m01"] / cubeMoment["m00"])

                # # find if center is in world frame
                centerCoordInWorld = np.matmul(self.convert_to_world, [centerX,centerY,1])[:]
            
                if centerCoordInWorld[0] < 0 or centerCoordInWorld[0] > 608 or centerCoordInWorld[1] < 0 or centerCoordInWorld[1] > 603.25 or (220 < centerCoordInWorld[0] and centerCoordInWorld[0] < 390 and 220 < centerCoordInWorld[1] and centerCoordInWorld[1] < 375):
                    continue
            
                # color detection points array
                colorDetectionPoints = [(centerX-3,centerY-3), (centerX-3,centerY-2), (centerX-3,centerY-1), (centerX-3,centerY), (centerX-3,centerY+1), (centerX-3,centerY+2), (centerX-3,centerY+3), 
                    (centerX-2,centerY-3), (centerX-2,centerY-2), (centerX-2,centerY-1), (centerX-2,centerY), (centerX-2,centerY+1), (centerX-2,centerY+2), (centerX-2,centerY+3), 
                    (centerX-1,centerY-3), (centerX-1,centerY-2), (centerX-1,centerY-1), (centerX-1,centerY), (centerX-1,centerY+1), (centerX-1,centerY+2), (centerX-1,centerY+3), 
                    (centerX,centerY-3), (centerX,centerY-2), (centerX,centerY-1), (centerX,centerY), (centerX,centerY+1), (centerX,centerY+2), (centerX,centerY+3), 
                    (centerX+1,centerY-3), (centerX+1,centerY-2), (centerX+1,centerY-1), (centerX+1,centerY), (centerX+1,centerY+1), (centerX+1,centerY+2), (centerX+1,centerY+3),
                    (centerX+2,centerY-3), (centerX+2,centerY-2), (centerX+2,centerY-1), (centerX+2,centerY), (centerX+2,centerY+1), (centerX+2,centerY+2), (centerX+2,centerY+3),
                    (centerX+3,centerY-3), (centerX+3,centerY-2), (centerX+3,centerY-1), (centerX+3,centerY), (centerX+3,centerY+1), (centerX+3,centerY+2), (centerX+3,centerY+3),
                    ]
                hSum = 0
                sSum = 0
                vSum = 0
                hAve = 0
                sAve = 0
                vAve = 0
                len1 = 0

                for k in range(len(colorDetectionPoints)):
                    # find hsv
                    h = self.hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][0]
                    s = self.hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][1]
                    v = self.hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][2]
                    hSum = hSum + h
                    sSum = sSum + s
                    vSum = vSum + v
                hAve = hSum/len(colorDetectionPoints)
                sAve = sSum/len(colorDetectionPoints)
                vAve = vSum/len(colorDetectionPoints)

                if hAve >= lower[0] and hAve <= upper[0] and sAve >= lower[1] and sAve <= upper[1] and vAve >= lower[2] and vAve <= upper[2]:
                    
                    # approximate bounding rectangle
                    rect = cv2.minAreaRect(self.contoursByDepth[i])
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    # judge if contour is cube # cube length in mouse coord: at least 19
                    len1 = ((box[0][0]-box[1][0])**2+(box[0][1]-box[1][1])**2)**0.5
                    len2 = ((box[0][0]-box[3][0])**2+(box[0][1]-box[3][1])**2)**0.5
                    if (int(len1) < 21) or (int(len2) < 21) or (int(len1) > 40) or (int(len2) > 40):
                        continue

                    # record vertexs
                    self.rectVertex.append(box)
                    # vertex coords in world frame
                    vertexCoord1 = np.matmul(self.convert_to_world, [box[0][0],box[0][1],1])
                    vertexCoord2 = np.matmul(self.convert_to_world, [box[1][0],box[1][1],1])
                    # self.vertexCoordInWorld.append([vertexCoord1,vertexCoord2])
                    # record orientation in world frame
                    angle = np.arctan((vertexCoord2[1]-vertexCoord1[1])/(vertexCoord2[0]-vertexCoord1[0]))
                    self.cubeOrient.append(angle)
                    # define colors
                    self.detectedCubeColor.append(cubeColor[j])
                    # record contours
                    self.cubeContours.append(self.contoursByDepth[i])
                    # record center coords
                    self.cubeCenter.append([int(centerCoordInWorld[0]),int(centerCoordInWorld[1])])
                    camera_coord.append([centerX,centerY])
                else:
                    continue

        if(self.blockMessage or True):
            print "kinect msg"
            print self.cubeCenter
            print self.cubeOrient
            print self.detectedCubeColor
            print camera_coord

        return camera_coord


    def detectBlocksInDepthImage(self,boundMin, boundMax):
        """
        Implement a blob detector to find blocks
        in the depth image
        """
        self.captureDepthFrame()
        print "bound: ", boundMin, boundMax
        contoursDepth = np.array([])
        # convert depthImage into 8 bits
        depthImage = self.currentDepthFrame[:]
        np.clip(depthImage,0,2**10 - 1,depthImage)
        depthImage >>= 2
        depthImage = depthImage.astype(np.uint8)

        # load rgb image produce hsv image 
        self.rgbImage = self.currentVideoFrame
        self.rgbImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2RGB)
        self.hsvImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2HSV)
        # use grayscale in depth to measure the depth of object

        # detect object
        grayBoundaries = [(boundMin, boundMax)]
            # (160,169)] # 3st layer
            # (170,173), # 2st layer
            # (174,177)] # 1st layer

        for i in range(len(grayBoundaries)):
            (grayLower,grayUpper) = grayBoundaries[i]
            grayLower = np.array(grayLower,dtype = "uint8")
            grayUpper = np.array(grayUpper,dtype = "uint8")
            grayThreshold = cv2.inRange(depthImage,grayLower,grayUpper)
            # dilation
            kernel = np.ones((5,5),np.uint8)
            grayDilation = cv2.dilate(grayThreshold,kernel,iterations = 1)
            # find countors
            _, contours, _ = cv2.findContours(grayDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            contoursDepth = np.append(contoursDepth, contours)

        print "in detectBlocksInDepthImage, contour depth has length", len(contoursDepth)
        return contoursDepth

    # takes in world coordinates and returns depth of that point
    def depthOf(self,x,y):
        world_coord = [x,y,1]
        camera_coord = np.matmul(self.convert_to_cam, world_coord)
        print "depthOf"
        print camera_coord
        cam_Z = self.currentDepthFrame[int(camera_coord[1])][int(camera_coord[0])]
        return self.worldHeight - 0.1236 * 1000 * np.tan(cam_Z/2842.5 + 1.1863)

