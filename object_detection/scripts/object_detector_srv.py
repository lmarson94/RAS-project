#!/usr/bin/env python

import rospy
import roslib
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
#from skimage.measure import regionprops, label
#from skimage.color import label2rgb
from object_detection.msg import msg_objects_detected
from object_detection.srv import DetectObjectsImage

#This class does the filtering and creates the bounding boxes
class image_detector:

	def __init__(self):
		self.bound_ls = []
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
		lower = np.array([-1, 200, 80])# lower = np.array([0, 220, 100])
		upper = np.array([3, 255, 255])# upper = np.array([3, 255, 200])
		self.bound_ls.append((lower, upper, onlyS, "Red"))
		#Red - other side
		onlyS = False;
		lower = np.array([155, 190, 80])# lower = np.array([160, 220, 100])
		upper = np.array([190, 255, 255])# upper = np.array([180, 255, 200])
		self.bound_ls.append((lower, upper, onlyS, "Red"))
		#Yellow
		onlyS = False;
		lower = np.array([15, 190, 100])
		upper = np.array([21, 255, 255])
		self.bound_ls.append((lower, upper, onlyS, "Yellow"))
		#Green
		onlyS = False;
		lower = np.array([45, 50, 30])
		upper = np.array([85, 255, 240])
		self.bound_ls.append((lower, upper, onlyS, "Green"))

		self.kernel1 = np.ones((5,5), np.uint8)
		self.kernel2 = np.ones((3,3), np.uint8)

		self.dict = {	"Purple": 0,
				"Orange": 1,
				"Blue": 2,
				"Red": 3,
				"Yellow": 4,
				"Green": 5,	}
	def checkProp(self, dx, dy):
	  prop = float(dx)/float(dy)
	  if (prop < 1.2 and prop > 0.8):
	    return True
	  else:
	    return False
	def detect_objects(self, frame):
		msg = msg_objects_detected()

		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

		for bound in self.bound_ls:
			if bound[2] == True: # not needed at the moment.
				img_s = hsv[:, :, 1]

				ret, bw = cv2.threshold(img_s, 190, 255, cv2.THRESH_BINARY)

				im, contours, hierarchy = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
				for c in contours:
					m = cv2.moments(c)
					Area = m['m00']
					if Area > 300:
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
							int(center[0]), int(center[1]), self.dict.get(bound[3])])

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

#Overall class that organizes the other classes and the main node
class object_detector:
	def __init__(self):
		#setting variables
		rospy.init_node('object_detector', anonymous=True)
		#self.r = rospy.Rate(10)
		self.service = rospy.Service('detect_objects_image', DetectObjectsImage, self.detect_objects_image)
		self.img_det = image_detector()
		self.bridge = CvBridge()

	def detect_objects_image(self, inputMsg):
		rospy.loginfo("Service for Object Detection called.")

		#np_arr = np.fromstring(data.data, np.uint8)
		#cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		#cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
		try:
			cv_image = self.bridge.imgmsg_to_cv2(inputMsg.inputImage, "rgb8")
			(rows, cols, channels) = cv_image.shape
			if cols > 60 and rows > 60:
				image_out, msg = self.img_det.detect_objects(cv_image) #image out is not needed here anymore...
		except CvBridgeError as e:
			rospy.loginfo("Error in CvBridge Sub.")
			msg = msg_objects_detected()

		return msg

	def object_detection(self):
		try:
			rospy.spin()
		except KeyboardInterrupt:
			rospy.loginfo("Shutting down.")

if __name__=='__main__':
	obj_dec = object_detector()
	obj_dec.object_detection()
