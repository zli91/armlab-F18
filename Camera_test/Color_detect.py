import cv2
import sys

font = cv2.FONT_HERSHEY_SIMPLEX

def mouse_callback(event,x,y,flags,param):
	r = img[y][x][2]
	g = img[y][x][1]
	b = img[y][x][0]
	h = hsv[y][x][0]
	s = hsv[y][x][1]
	v = hsv[y][x][2]
	output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
	output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
	tmp = img.copy()
	cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
	cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
	cv2.imshow('window', tmp)
	if event == cv2.EVENT_LBUTTONDOWN:
		print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)

if len(sys.argv) == 2:
	print "Opening " + str(sys.argv[1])
	img = cv2.imread(sys.argv[1])
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	cv2.namedWindow("window",1)
	cv2.imshow('window', img)
	cv2.setMouseCallback("window",mouse_callback)

	while True:
		ch = 0xFF & cv2.waitKey(10)
		if ch == 27:
			break
	cv2.destroyAllWindows()

else:
	print "Expected filename as argument"

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






