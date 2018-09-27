import cv2
import sys
import numpy as np
import freenect

font = cv2.FONT_HERSHEY_SIMPLEX

def mouse_callback(event,x,y,flags,param):
	r = rgbImg[y][x][2]
	g = rgbImg[y][x][1]
	b = rgbImg[y][x][0]
	h = hsv[y][x][0]
	s = hsv[y][x][1]
	v = hsv[y][x][2]
	output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
	output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
	tmp = rgbImg.copy()
	cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
	cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
	cv2.imshow('window', tmp)
	if event == cv2.EVENT_LBUTTONDOWN:
		print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)
		# print "bgr: (%d, %d, %d)" % (b,g,r)

if len(sys.argv) == 1:
	# rgb value
    rgbImg = freenect.sync_get_video()[0]
    rgbImg = cv2.cvtColor(rgbImg, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(rgbImg, cv2.COLOR_BGR2HSV)
    cv2.namedWindow("window",1)
    cv2.imshow('window', rgbImg)
    cv2.setMouseCallback("window",mouse_callback)
    while True:
		ch = 0xFF & cv2.waitKey(10)
		if ch == 27:
			break
	# cv2.destroyAllWindows()

else:
	print "Expected filename as argument"








