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
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


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
        print result
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
            ([2, 101, 236], [30, 156, 254]), # yellow
            ([7, 185, 20], [12, 240, 66]), # orange
            ([168, 152, 232], [175, 172, 246]), # pink
            ([19, 5, 60], [178, 61, 79]), # black
            ([171, 161, 172], [178, 201, 186]), # red
            ([146, 60, 133], [162, 80, 154]), # purple
            ([52, 85, 134],[80, 119, 151]), # green
            ([110, 97, 159], [132, 126, 171]) # blue
            ]
        ### color detection in rgb image
        # r = self.rgbImage[centerY][centerX][2]
        # g = self.rgbImage[centerY][centerX][1]
        # b = self.rgbImage[centerY][centerX][0]
        # print len(self.cubeContours)
        self.cubeCenter = []
        self.detectedCubeColor = []
        for i in range(len(self.cubeContours)):        
            # find center of mass
            cubeMoment = cv2.moments(self.cubeContours[i])
            centerX = int(cubeMoment["m10"] / cubeMoment["m00"])
            centerY = int(cubeMoment["m01"] / cubeMoment["m00"])
            # find nearby point
            
            # find hsv
            h = self.hsvImage[centerY][centerX][0]
            s = self.hsvImage[centerY][centerX][1]
            v = self.hsvImage[centerY][centerX][2]

            for j in range(len(hsvBoundaries)):
                (lower,upper) = hsvBoundaries[j]
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                if h >= lower[0] and h <= upper[0] and s >= lower[1] and s <= upper[1] and v >= lower[2] and v <= upper[2]:
                    # define colors
                    self.detectedCubeColor.append(cubeColor[j])
                    # draw contours
                    cv2.drawContours(self.currentVideoFrame,[self.cubeContours[i]],-1,(0,255,0),3)
                    # record coords
                    self.cubeCenter.append([centerX,centerY])
                else:
                    continue
        return self.cubeCenter, self.detectedCubeColor



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
        # use grayscale in depth image to detect object
        (grayLower,grayUpper) = (150, 178)
        grayLower = np.array(grayLower,dtype = "uint8")
        grayUpper = np.array(grayUpper,dtype = "uint8")
        grayThreshold = cv2.inRange(depthImage,grayLower,grayUpper)
        # dilation
        kernel = np.ones((5,5),np.uint8)
        grayDilation = cv2.dilate(grayThreshold,kernel,iterations = 1)
        # find countors
        _, self.cubeContours, _ = cv2.findContours(grayDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        return None



