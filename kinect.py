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
        self.cubeContours = np.array([])
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
        self.worldHeight = 945
        self.x_off = 304.88  # distances from center of the bottom of ReArm to world origin
        self.y_off = 301.5

        self.blockDetected = False

        self.blockMessage = False

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
        # check if cv is calibrated
        if (self.kinectCalibrated == True):
            z = self.currentDepthFrame[y][x]
        else:
            print("ERROR: Camera Calibrate should be completed prior to Click and Grab")
            return
        # calculate the coordinates
        mouse_coor = [x,y,1]
        world_coord = np.matmul(self.convert_to_world, mouse_coor)
        # x and y coordinates converting to board without pinhole correction
        cam_X = world_coord[0]
        cam_Y = world_coord[1]
        # actual dheight above board of current point
        world_Z = self.worldHeight - 0.1236 * 1000 * np.tan(z/2842.5 + 1.1863)
        # converting x in pinhold to x in world
        d = (self.x_off-cam_X)*(self.worldHeight-cam_Z)/self.worldHeight
        diff = (self.x_off-cam_X)-d
        world_X = cam_X+diff;
        # converting y in pinhole to y in world
        d = (self.y_off-cam_Y)*(self.worldHeight-cam_Z)/self.worldHeight
        diff = (self.y_off-cam_Y)-d
        world_Y = cam_Y+diff;
        
        return [world_X, world_Y, world_Z]



    def processVideoFrame(self):
        self.detectBlocksInDepthImage()
        self.blockDetector()
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
            self.DepthHSV[...,0] = self.currentDepthFrame
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
    
    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """

        cubeColor = ['yellow','orange','pink','black','red','purple','green','blue']
        # rgbBoundaries = [ # b,g,r
        #     ([4, 160, 240], [50, 210, 253]), # yellow
        #     ([10, 85, 195], [30, 130, 205]), # orange
        #     ([85, 48, 189], [110, 60, 210]), # pink
        #     ([10, 21, 21], [50, 30, 38]), # black
        #     ([26, 22, 140], [60, 38, 160]), # red
        #     ([110, 65, 120], [127, 86, 140]), # purple
        #     ([94, 120, 87],[108, 130, 105]), # green
        #     ([130, 95, 80], [158, 99, 85]) # blue
        #     ]
        hsvBoundaries = [ # h,s,v
            ([20, 30, 101], [40, 255, 255]), # yellow
            ([0, 30, 101], [15, 255, 255]), # orange
            ([160, 90, 230], [180, 170, 255]), # pink
            ([0, 0, 70], [255, 255, 100]), # black
            ([160, 165, 140], [180, 255, 200]), # red
            ([120, 20, 130], [155, 130, 220]), # purple
            ([45, 50, 120],[80, 120, 170]), # green
            ([100, 100, 160], [130, 190, 200]) # blue
            ]

        ### color detection in rgb image
        # r = self.rgbImage[centerY][centerX][2]
        # g = self.rgbImage[centerY][centerX][1]
        # b = self.rgbImage[centerY][centerX][0]

        self.cubeCenter = []
        self.detectedCubeColor = []
        colorDetectionPoints = []
        self.cubeContours = []
        self.rectVertex = []
        self.cubeOrient = []
        self.vertexCoordInWorld = []
        
        for i in range(len(self.contours)):        
            # find center of mass
            cubeMoment = cv2.moments(self.contours[i])
            centerX = int(cubeMoment["m10"] / cubeMoment["m00"])
            centerY = int(cubeMoment["m01"] / cubeMoment["m00"])

            # # find if center is in world frame
            centerCoordInWorld = np.matmul(self.convert_to_world, [centerX,centerY,1])
            if centerCoordInWorld[0] < 0 or centerCoordInWorld[0] > 608 or centerCoordInWorld[1] < 0 or centerCoordInWorld[1] > 603.25 or (220 < centerCoordInWorld[0] < 375 and 250 < centerCoordInWorld[1] < 390):
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

            for j in range(len(hsvBoundaries)):
                (lower,upper) = hsvBoundaries[j]
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                if hAve >= lower[0] and hAve <= upper[0] and sAve >= lower[1] and sAve <= upper[1] and vAve >= lower[2] and vAve <= upper[2]:
                    # define colors
                    self.detectedCubeColor.append(cubeColor[j])
                    # record contours
                    self.cubeContours.append(self.contours[i])
                    # record center coords
                    self.cubeCenter.append([int(centerCoordInWorld[0]),int(centerCoordInWorld[1])])
                    # approximate bounding rectangle
                    rect = cv2.minAreaRect(self.contours[i])
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    # record vertexs
                    self.rectVertex.append(box)
                    # vertex coords in world frame
                    vertexCoord1 = np.matmul(self.convert_to_world, [box[0][0],box[0][1],1])
                    vertexCoord2 = np.matmul(self.convert_to_world, [box[1][0],box[1][1],1])
                    # self.vertexCoordInWorld.append([vertexCoord1,vertexCoord2])
                    # record orientation in world frame
                    angle = np.arctan((vertexCoord2[1]-vertexCoord1[1])/(vertexCoord2[0]-vertexCoord1[0]))
                    self.cubeOrient.append(angle)

                else:
                    continue

            if(self.blockMessage):
                print self.cubeCenter
                print self.cubeOrient
                print self.detectedCubeColor
            

        return None


    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        # convert depthImage into 8 bits
        depthImage = self.currentDepthFrame
        np.clip(depthImage,0,2**10 - 1,depthImage)
        depthImage >>= 2
        depthImage = depthImage.astype(np.uint8)

        # load rgb image produce hsv image 
        self.rgbImage = self.currentVideoFrame
        self.rgbImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2RGB)
        self.hsvImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2HSV)
        # use grayscale in depth to measure the depth of object

        # detect object
        (grayLower,grayUpper) = (150, 178)
        grayLower = np.array(grayLower,dtype = "uint8")
        grayUpper = np.array(grayUpper,dtype = "uint8")
        grayThreshold = cv2.inRange(depthImage,grayLower,grayUpper)
        # dilation
        kernel = np.ones((5,5),np.uint8)
        grayDilation = cv2.dilate(grayThreshold,kernel,iterations = 1)
        # find countors
        _, self.contours, _ = cv2.findContours(grayDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


        return None



