#!/usr/bin/env python

import rospy
import roslib
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
#from skimage.measure import regionprops, label
#from skimage.color import label2rgb
from object_detection.msg import msg_objects_detected

#This class does the filtering and creates the bounding boxes
class image_detector:

	def __init__(self):
		self.bound_ls = []
		#Purple rgb(76, 56, 89)
		#onlyS = False;
		#purple = np.uint8([[[89, 56, 76]]]) #Color in BGR
		#hsv_purple = cv2.cvtColor(purple, cv2.COLOR_BGR2HSV)
		#print("Purple1: ", hsv_purple)
		#purple = np.uint8([[[138, 101, 117]]]) #Color in BGR
		#hsv_purple = cv2.cvtColor(purple, cv2.COLOR_BGR2HSV)
		#print("Purple2: ", hsv_purple)
		#lower_purple = np.array([hsv_purple[0, 0, 0] - 20, 85, 60])
		#upper_purple = np.array([hsv_purple[0, 0, 0] + 20, 255, 255])
		#self.bound_ls.append((lower_purple, upper_purple, onlyS, "Purple"))

		#Convert colors calib
		#rgb(154, 154, 158)
		#rgb(26, 134, 22) rgb(52, 118, 27)

		color = np.uint8([[[36, 73, 0]]]) #Color in BGR
		hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
		print("Color in hsv: ", hsv_color)

		#Purple
		onlyS = False
		lower = np.array([120, 40, 20]) #lower = np.array([125, 50, 40])
		upper = np.array([150, 160, 240]) #upper = np.array([145, 140, 240])
		self.bound_ls.append((lower, upper, onlyS, "Purple"))
		#Orange
		onlyS = False;
		lower = np.array([4, 225, 140])
		upper = np.array([11, 255, 240])
		self.bound_ls.append((lower, upper, onlyS, "Orange"))
		#Blue
		onlyS = False;
		lower = np.array([95, 120, 40])
		upper = np.array([105, 255, 200])
		self.bound_ls.append((lower, upper, onlyS, "Blue"))
		#Red
		onlyS = False;
		lower = np.array([-1, 210, 80])# lower = np.array([0, 220, 100])
		upper = np.array([3, 255, 220])# upper = np.array([3, 255, 200])
		self.bound_ls.append((lower, upper, onlyS, "Red"))
		#Red - other side
		onlyS = False;
		lower = np.array([155, 210, 80])# lower = np.array([160, 220, 100])
		upper = np.array([185, 255, 220])# upper = np.array([180, 255, 200])
		self.bound_ls.append((lower, upper, onlyS, "Red"))
		#Yellow
		onlyS = False;
		lower = np.array([15, 200, 100])
		upper = np.array([21, 255, 255])
		self.bound_ls.append((lower, upper, onlyS, "Yellow"))
		#Green
		onlyS = False;
		lower = np.array([50, 70, 30])
		upper = np.array([85, 255, 230])
		self.bound_ls.append((lower, upper, onlyS, "Green"))

		self.kernel1 = np.ones((5,5), np.uint8)
		self.kernel2 = np.ones((3,3), np.uint8)
	def checkProp(self, dx, dy):
	  prop = float(dx)/float(dy)
	  if (prop < 1.2 and prop > 0.8):
	    return True
	  else:
	    return False
	def detect_objects(self, frame):
		msg = msg_objects_detected()

		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
		font = cv2.FONT_HERSHEY_SIMPLEX

		for bound in self.bound_ls:
			if bound[2] == True:
				img_s = hsv[:, :, 1]

				ret, bw = cv2.threshold(img_s, 190, 255, cv2.THRESH_BINARY)

				im, contours, hierarchy = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
				for c in contours:
					m = cv2.moments(c)
					Area = m['m00']
					if Area > 250:
						x, y, dx, dy = cv2.boundingRect(c)
						center = (m['m10']/Area, m['m01']/Area)
						cv2.rectangle(frame, (x, y), (x+dx, y+dy), (0, 255, 0), 2)
						msg.coords.extend([int(x),int(y),int(x+dx),int(y+dy),
							int(center[0]), int(center[1])])

			else:
				mask = cv2.inRange(hsv, bound[0], bound[1])

				mask = cv2.dilate(mask, self.kernel1, iterations=1)
				mask = cv2.erode(mask, self.kernel1, iterations=1)
				mask = cv2.dilate(mask, self.kernel2, iterations=1)
			  	im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		  		for c in contours:
				  m = cv2.moments(c)
				  Area = m['m00']
				  if Area > 250:
					  x, y, dx, dy = cv2.boundingRect(c)
					  if (self.checkProp(dx, dy)): #only allow bb which are roughly 1:1
					    center = (m['m10']/Area, m['m01']/Area)
					    cv2.rectangle(frame, (x, y), (x+dx, y+dy), (0, 255, 0), 2)
					    msg.coords.extend([int(x),int(y),int(x+dx),int(y+dy),
						    int(center[0]), int(center[1])])
					    cv2.putText(frame, bound[3], (x+dx, y+dy), font, 0.8, (0,0,0), 2, cv2.LINE_AA)

        #kernel = np.ones((5,5), np.uint8)
        #mask = cv2.dilate(mask, kernel, iterations = 3)

		##This method uses regionprops ... maximum rate ~ 3-5Hz
		#label_image = label(bw)
		#image_label_overlay = label2rgb(label_image, image = frame)
		#for region in regionprops(label_image):
		#	if region.area >= 300:
		#		#print(region.centroid)
		#		y1,x1,y2,x2 = region.bbox
		#		cv2.rectangle(frame, (x1,y1), (x2, y2), (0,255,0), 3)
		#		msg.coords.extend([int(x1),int(y1),int(x2),int(y2),int(region.centroid[0]), int(region.centroid[1])])
		return frame, msg

#This class is a bridge to convert input images to opencv_images and vice versa
class image_converter:
	def __init__(self):
		self.image_pub = rospy.Publisher("image_out", Image, queue_size=1)
		self.pub = rospy.Publisher("objects_detected", msg_objects_detected, queue_size=1)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color/compressed", CompressedImage, self.callback)

		self.img_det = image_detector()

	def callback(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
		# try:
		# 	cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
		# except CvBridgeError as e:
		# 	rospy.loginfo("Error in CvBridge Sub.")
		#
		(rows, cols, channels) = cv_image.shape
		if cols > 60 and rows > 60:
		 	image_out, msg = self.img_det.detect_objects(cv_image)
		# 	self.pub.publish(msg)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_out, "rgb8"))
		except CvBridgeError as e:
		 	rospy.loginfo("Error in CvBridge Pub.")

#Overall class that organizes the other classes and the main node
class object_detector:
	def __init__(self):
		#setting variables
		rospy.init_node('object_detector', anonymous=True)
		#self.r = rospy.Rate(10)
		self.ic = image_converter()

	def object_detection(self):
		try:
			rospy.spin()
		except KeyboardInterrupt:
			rospy.loginfo("Shutting down.")

if __name__=='__main__':
	obj_dec = object_detector()
	obj_dec.object_detection()
