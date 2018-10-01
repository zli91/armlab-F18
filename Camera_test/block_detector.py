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
        ([20, 220, 240], [25,255, 255]), # yellow
        ([5, 200, 195], [15, 250, 205]), # orange
        ([170, 180, 190], [174, 194, 205]), # pink
        ([0, 20, 25], [180, 150, 50]), # black
        ([175, 195, 140], [180, 210, 155]), # red
        ([150, 90, 120], [165, 105, 135]), # purple
        ([50, 45, 115],[65, 60, 130]), # green
        ([110, 110, 140], [120, 125, 155]) # blue
        ]
        ## color detection in rgb image
        
    # initial info of cube center coord & color
    cubeCenter = []
    detectedCubeColor = []
    colorDetectionPoints = []
    rectVertex2 = []

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

        # # find rgb
        # r = rgbImage[centerY][centerX][2]
        # g = rgbImage[centerY][centerX][1]
        # b = rgbImage[centerY][centerX][0]

        for k in range(len(colorDetectionPoints)):
            # find hsv
            h = hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][0]
            s = hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][1]
            v = hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][2]
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
                # approximate bounding rectangle
                rect = cv2.minAreaRect(cubeContours[i])
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                # record vertexs
                rectVertex2.append([box[1],box[2]])
                
                # draw contours
                cv2.drawContours(rgbImage,[box],-1,(0,255,0),3)
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

