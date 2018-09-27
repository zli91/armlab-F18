import cv2
import numpy as np
import sys

# load the images
depthImage = cv2.imread(sys.argv[1])
rgpImage = cv2.imread(sys.argv[2])
hsvImage = cv2.cvtColor(rgpImage, cv2.COLOR_BGR2HSV)

### cube detection in depth image
(grayLower,grayUpper) = (0, 177)
grayLower = np.array(grayLower,dtype = "uint8")
grayUpper = np.array(grayUpper,dtype = "uint8")
grayThreshold = cv2.inRange(depthImage,grayLower,grayUpper)

# Morpholigical Operations
kernel = np.ones((5,5),np.uint8)
grayDilation = cv2.dilate(grayThreshold,kernel,iterations = 1)
# find countors
_, cubeContours, _ = cv2.findContours(grayDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

cubeColor = ['yellow','orange','pink','black','red','purple','green','blue']
rgbBoundaries = [ # b,g,r
    ([4, 160, 240], [50, 210, 253]), # yellow
    ([10, 85, 195], [30, 130, 205]), # orange
    ([85, 48, 189], [110, 60, 210]), # pink
    ([10, 21, 21], [50, 30, 38]), # black
    ([26, 22, 140], [60, 38, 160]), # red
    ([110, 65, 120], [127, 86, 140]), # purple
    ([94, 120, 87],[108, 130, 105]), # green
    ([130, 95, 80], [158, 99, 85]) # blue
    ]
for i in range(len(cubeContours)):
    # draw contours
    cv2.drawContours(rgpImage,[cubeContours[i]],-1,(0,255,0),3)
    # visualization
    cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
    cv2.imshow('window', rgpImage)
    cv2.waitKey(0)
    # find moment
    cubeMoment = cv2.moments(cubeContours[i])
    centerX = int(cubeMoment["m10"] / cubeMoment["m00"])
    centerY = int(cubeMoment["m01"] / cubeMoment["m00"])
    print centerX,centerY
    ### color detection in rgb image
    r = rgpImage[centerY][centerX][2]
    g = rgpImage[centerY][centerX][1]
    b = rgpImage[centerY][centerX][0]
    print b,g,r
    h = hsvImage[centerY][centerX][0]
    s = hsvImage[centerY][centerX][1]
    v = hsvImage[centerY][centerX][2]
    for j in range(len(rgbBoundaries)):
        (lower,upper) = rgbBoundaries[j]
        if (b,g,r) >= lower and (b,g,r) <= upper:
            print cubeColor[j]
        else:
            continue



    
    






# pirnt(sys.argv[3])
# # print(cubeColor[0])
# # for i in cubeColor:

# if sys.argv[3] == cubeColor[i]:
#     (lower,upper) = colorBoundaries[i]
#     lower = np.array(lower,dtype = "uint8")
#     upper = np.array(upper,dtype = "uint8")
#     # Thresholding
#     colorThreshold = cv2.inRange(image,lower,upper)
#     # Morpholigical Operations
#     kernel = np.ones((5,5),np.uint8)
#     colorDilation = cv2.dilate(colorThreshold,kernel,iterations = 1)
#     # find countors
#     _, colorContours, _ = cv2.findContours(colorDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#     # draw contours
#     cv2.drawContours(image,colorContours,-1,(0,255,0),3)
#     # find moment
#     cnt = colorContours[0]
#     colorCubeMoment = cv2.moments(cnt)
#     # visualization
#     cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
#     cv2.imshow('window', image)
#     cv2.waitKey(0)
# else:
#     print("No cube found!")

