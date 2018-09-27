import cv2
import numpy as np
import sys

def blockDetector(depthImage,rgbImage,hsvImage):
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
        ## color detection in rgb image
        
    # initial info of cube center coord & color
    cubeCenter = []
    detectedCubeColor = []
    colorDetectionPoints = []
    hSum = 0
    sSum = 0
    vSum = 0

    # use threshold to detect blocks in depth image
    (grayLower,grayUpper) = (150, 178)
    grayLower = np.array(grayLower,dtype = "uint8")
    grayUpper = np.array(grayUpper,dtype = "uint8")
    grayThreshold = cv2.inRange(depthImage,grayLower,grayUpper)
        
    # dilation
    kernel = np.ones((5,5),np.uint8)
    grayDilation = cv2.dilate(grayThreshold,kernel,iterations = 1)
    # find countors
    _, cubeContours, _ = cv2.findContours(grayDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    for i in range(len(cubeContours)):        
        # find center of mass
        cubeMoment = cv2.moments(cubeContours[i])
        centerX = int(cubeMoment["m10"] / cubeMoment["m00"])
        centerY = int(cubeMoment["m01"] / cubeMoment["m00"])
        # color detection points array
        colorDetectionPoints = [(centerX,centerY), 
            (centerX+3,centerY), 
            (centerX,centerY+3), 
            (centerX-3,centerY), 
            (centerX,centerY-3),]
        
        # # find rgb
        # r = rgbImage[centerY][centerX][2]
        # g = rgbImage[centerY][centerX][1]
        # b = rgbImage[centerY][centerX][0]

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
            # if r >= lower[2] and r <= upper[2] and g >= lower[1] and g <= upper[1] and b >= lower[0] and b <= upper[0] :
            if hAve >= lower[0] and hAve <= upper[0] and sAve >= lower[1] and sAve <= upper[1] and vAve >= lower[2] and vAve <= upper[2]:
                # define colors
                detectedCubeColor.append(cubeColor[j])
                # draw contours
                cv2.drawContours(rgbImage,[cubeContours[i]],-1,(0,255,0),3)
                # record coords
                cubeCenter.append([centerX,centerY])
            else:
                continue
    return cubeCenter, detectedCubeColor



# def detectBlocksInDepthImage():

#     (grayLower,grayUpper) = (150, 178)
#     grayLower = np.array(grayLower,dtype = "uint8")
#     grayUpper = np.array(grayUpper,dtype = "uint8")
#     grayThreshold = cv2.inRange(depthImage,grayLower,grayUpper)
        
#     # Morpholigical Operations
#     kernel = np.ones((5,5),np.uint8)
#     grayDilation = cv2.dilate(grayThreshold,kernel,iterations = 1)
#     # find countors
#     _, self.cubeContours, _ = cv2.findContours(grayDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#     # cv2.drawContours(self.currentVideoFrame,self.cubeContours,-1,(0,255,0),3)
#     return None

if __name__ == '__main__':
    depthImage = cv2.imread(sys.argv[1])
    rgbImage = cv2.imread(sys.argv[2])
    hsvImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2HSV)
    (cubeCenter, detectedCubeColor) = blockDetector(depthImage,rgbImage,hsvImage)
    cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
    cv2.imshow('window', rgbImage)
    cv2.waitKey(0)
    print cubeCenter
    print detectedCubeColor

