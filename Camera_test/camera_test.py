# import cv2
# import sys
# import cv2

# font = cv2.FONT_HERSHEY_SIMPLEX

# def mouse_callback(event,x,y,flags,param):
# 	r = img[y][x][2]
# 	g = img[y][x][1]
# 	b = img[y][x][0]
# 	h = hsv[y][x][0]
# 	s = hsv[y][x][1]
# 	v = hsv[y][x][2]
# 	output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
# 	output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
# 	tmp = img.copy()
# 	cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
# 	cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
# 	cv2.imshow('window', tmp)
# 	if event == cv2.EVENT_LBUTTONDOWN:
# 		print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)

# if len(sys.argv) == 2:
# 	print "Opening " + str(sys.argv[1])
# 	img = cv2.imread(sys.argv[1])
# 	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 	cv2.namedWindow("window",1)
# 	cv2.imshow('window', img)
# 	cv2.setMouseCallback("window",mouse_callback)

# 	while True:
# 		ch = 0xFF & cv2.waitKey(10)
# 		if ch == 27:
# 			break
# 	cv2.destroyAllWindows()

# else:
# 	print "Expected filename as argument"





# import cv2
# import numpy as np
# import freenect

# depth_frame = freenect.sync_get_depth()[0]

# np.clip(depth_frame,0,2**10 - 1,depth_frame)
# depth_frame >>= 2
# depth_frame = depth_frame.astype(np.uint8)

# cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
# cv2.imshow('window', depth_frame)

# while True:
# 	ch = 0xFF & cv2.waitKey(10)
# 	if ch == 0x1B:
# 		break
# cv2.destroyAllWindows()






import cv2
import numpy as np
import argparse

# def blockDetector(self):
#         """
#         TODO:
#         Implement your block detector here.  
#         You will need to locate
#         blocks in 3D space
#         """
        # construct the argument parse
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())
    
        # load the image
image = cv2.imread(args["image"])

colorBoundaries = [
    ([4, 160, 240], [50, 210, 253]), # yellow
    ([10, 85, 195], [30, 130, 205]), # orange
    ([85, 48, 189], [110, 60, 210]), # pink
    ([10, 21, 21], [50, 30, 38]), # black
    ([26, 22, 140], [60, 38, 160]), # red
    ([110, 65, 120], [127, 86, 140]), # purple
    ([94, 120, 87],[108, 130, 105]), # green
    ([130, 95, 80], [158, 99, 85]) # blue

]
for (lower,upper) in colorBoundaries:
    lower = np.array(lower,dtype = "uint8")
    upper = np.array(upper,dtype = "uint8")
            # Thresholding
    colorThreshold = cv2.inRange(image,lower,upper)
    

    # ret,thresh = cv2.threshold(image,127,255,0)
    # print thresh

            # Morpholigical Operations
    kernel = np.ones((5,5),np.uint8)
    colorDilation = cv2.dilate(colorThreshold,kernel,iterations = 1)
    imgray = cv2.cvtColor(colorDilation,cv2.COLOR_BGR2GRAY)
    # cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('window', colorDilation)
    # cv2.waitKey(0)
    # print colorDilation
            # find countors
    colorContours, hierarchy = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # find moment
    colorCubeMoment = cv2.moments(colorContours)
            # draw the contours
    cnt = colorContours[4]
    cv2.drawContours(image,[cnt],0,(0,255,0),3)