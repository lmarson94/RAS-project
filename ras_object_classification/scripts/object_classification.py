#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from ras_object_classification.srv import ClassifyImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from keras.applications.mobilenet import preprocess_input #use this for mobilenet network
from keras.applications.vgg16 import preprocess_input #use this for vgg16 network

from keras.models import load_model
import os, rospkg
from PIL import Image #for testing only

class NeuronalNetwork:
	def __init__(self):
		self.model = load_model(os.path.join(rospkg.RosPack().get_path("ras_object_classification"), 'scripts', 'vgg_dropout_v3.h5')) # objects_nasnet_data_gen_dir_VX5.h5 # objects_vgg16_data_gen_dir_V4.h5 # vgg16_127-5_dataset2_v1.h5
		self.model._make_predict_function() # super important !!
		self.test_image = Image.open(os.path.join(rospkg.RosPack().get_path("ras_object_classification"), 'scripts', 'blue.jpg'))
		self.image_size = (224, 224)
		self.dict = {0:1, 1:10, 2:11, 3:12, 4:13, 5:14, 6:2, 7:3, 8:4, 9:5, 10:6, 11:7, 12:8, 13:9}
		#dict maps nn-answers to object type.
	def find_pred(self, answers):
		max_conf = 0.0
		index = 0
		print(answers)
		for i in range (0, 14):
			if (answers[:,i][0] > max_conf):
				max_conf = answers[:,i][0]
				index = i
		return max_conf, index
	def classify(self, image):
		clas = 15
		image = Image.fromarray(image)
		#image = image.crop(crop_area)
		image = image.resize(self.image_size, Image.LANCZOS)
		image = np.asarray(image).reshape(1, self.image_size[0], self.image_size[1], 3)
		#image = image/127.5 #CHANGE ACCORDING TO NETWORK !!
		image = preprocess_input(image)
		conf, clas_net = self.find_pred(self.model.predict(image))
		clas = self.dict.get(clas_net, 15)
		print clas
		return conf*100, clas

class ObjectClassifier:
	def __init__(self):
		self.service = rospy.Service('classify_image', ClassifyImage, self.classify_image_service)
		self.bridge = CvBridge()

		self.nn = NeuronalNetwork()

	def classify_image_service(self, req):
		clas = 15
		conf = 0
		rospy.loginfo("Starting Image Classification")
		try:
			cv_image = self.bridge.imgmsg_to_cv2(req.inputImage, "rgb8")
		except CvBridgeError as e:
			rospy.loginfo("Error in CvBridge Sub.")

		(rows, cols, channels) = cv_image.shape
		if cols > 60 and rows > 60:
			#area_ex = 30 #maybe prohibit to crop over border?
			#crop_area = (req.ulx - area_ex, req.uly - area_ex, req.lrx + area_ex, req.lry + area_ex)
			conf, clas = self.nn.classify(cv_image)

		return clas, conf


	def object_classification(self):
		rospy.init_node('object_classifier')

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()

if __name__ == '__main__':
	obj_clas = ObjectClassifier()
	obj_clas.object_classification()
