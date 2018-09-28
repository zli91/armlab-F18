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

        self.blockDetected = False

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

        

    def processVideoFrame(self):
        # draw contours
        cv2.drawContours(self.currentVideoFrame,self.cubeContours,-1,(0,255,0),3)



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
            ([20, 0, 240], [40, 255, 255]), # yellow
            ([5, 200, 220], [15, 250, 250]), # orange
            ([160, 149, 230], [180, 172, 246]), # pink
            ([19, 5, 60], [178, 61, 79]), # black
            ([171, 161, 172], [178, 190, 186]), # red
            ([140, 60, 128], [152, 80, 140]), # purple
            ([52, 85, 134],[85, 120, 151]), # green
            ([110, 130, 180], [122, 170, 200]) # blue
            ]

        ### color detection in rgb image
        # r = self.rgbImage[centerY][centerX][2]
        # g = self.rgbImage[centerY][centerX][1]
        # b = self.rgbImage[centerY][centerX][0]

        self.cubeCenter = []
        self.detectedCubeColor = []
        colorDetectionPoints = []
        self.cubeContours = []
        
        for i in range(len(self.contours)):        
            # find center of mass
            cubeMoment = cv2.moments(self.contours[i])
            centerX = int(cubeMoment["m10"] / cubeMoment["m00"])
            centerY = int(cubeMoment["m01"] / cubeMoment["m00"])

            # # # find if center is in world frame
            # convertToWorld = np.array([])
            # world_coordinates = np.array([[0,0], [0,603.25], [608, 603.25], [608,0], [304, 301.625]])
            # worldAffine = self.getAffineTransform(self.rgb_click_points, world_coordinates, 4)
            # convertToWorld = np.append(worldAffine, [[0, 0, 1]], axis=0)
            # centerCoordInWorld = np.matmul(convertToWorld, [centerX,centerY,??????])
            # print centerCoordInWorld
            # if centerCoordInWorld[0] < 0 or centerCoordInWorld[0] > 608 or centerCoordInWorld[1] < 0 or centerCoordInWorld[1] > 603.25 :
            #     break

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
                    # record coords
                    self.cubeCenter.append([centerX,centerY])
                else:
                    continue
        print self.cubeCenter
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



