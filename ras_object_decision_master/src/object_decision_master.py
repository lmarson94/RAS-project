#!/usr/bin/env python

import rospy
import roslib
import numpy as np
import itertools
import operator
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ras_object_master.msg import ObjectMasterMsg
from ras_msgs.msg import RAS_Evidence

import scipy.cluster.hierarchy as hcluster
# import matplotlib.pyplot as plt

class MakeTheBestOfIt:
	def __init__(self, ls_clas, ls_conf):
		self.ls_clas = ls_clas
		self.ls_conf = ls_conf

	def find_3_top_occurences(self):
		dict = {}
		for clas in self.ls_clas:
			if clas in dict:
				dict[clas] = dict[clas] + 1
			else:
				dict[clas] = 1
		sorted_dict = sorted(dict.items(), key=operator.itemgetter(1))
		sorted_dict.reverse()
		return sorted_dict[:3]

	def find_conf_avg_of_clas(self, clas):
		temp = [x == clas for x in self.ls_clas]
		temp_conf = [item for item, con in zip(self.ls_conf, temp) if con == True]
		return sum(temp_conf)/len(temp_conf)

	def getClassificationResult(self):
		print("CLASSIFICATION RESULT")
		print(self.ls_clas)
		top3 = self.find_3_top_occurences()
		firstGuess = (top3[0][0], top3[0][1], self.find_conf_avg_of_clas(top3[0][0]))	#structure (clas, occur, conf)
		try:
			secondGuess = (top3[1][0], top3[1][1], self.find_conf_avg_of_clas(top3[1][0]))
		except IndexError as e:
			print("no second guess.")
			secondGuess = (-99, -99, -99)
			pass
		try:
			thirdGuess = (top3[2][0], top3[2][1], self.find_conf_avg_of_clas(top3[2][0]))
		except IndexError as e:
			print("no third guess.")
			thirdGuess = (-99, -99, -99)
			pass

		print(firstGuess)
		print(secondGuess)
		print(thirdGuess)
		if firstGuess[0] != 15:
			return firstGuess[0], firstGuess[2]

		elif firstGuess[0] == 15 and secondGuess[0] == -99: #only 'an_object' is detected - classification fails always to determine class.
			return 15, 0

		elif firstGuess[0] == 15 and secondGuess[0] != -99: #an_object is detected in first place but in second place there are a high amonout of good classifications of a certain object.
			if secondGuess[1] > 5 and secondGuess[2] > 85: # should have several occurences and robust confidence before accepted.
				return secondGuess[0], secondGuess[2]
			else:
				return 15, 0

class target:
	def __init__(self, x_pos, y_pos, t_clas, t_conf, t_id):
		self.x = x_pos
		self.y = y_pos
		self.clas = t_clas
		self.conf = t_conf
		self.id = t_id

class battery:
	def __init__(self, x_pos, y_pos, b_pose):
		self.x = x_pos
		self.y = y_pos
		self.pose = b_pose

class target_handler:
	def __init__(self):
		self.ls = []
		self.cluster_points = []

		self.id = 0
		self.threshold = 0.05

		self.targets = []
		self.target_pub = rospy.Publisher("/target/Geometry/New_Point", ObjectMasterMsg, queue_size=1)

		self.takeEveryVal = 1
		self.counter = 0

		self.currentIDCounter = 0

		self.classToName = [RAS_Evidence.yellow_ball, RAS_Evidence.yellow_cube,
	        RAS_Evidence.green_cube, RAS_Evidence.green_cylinder,
	        RAS_Evidence.green_hollow_cube, RAS_Evidence.orange_cross,
	        RAS_Evidence.patric, RAS_Evidence.red_cylinder, RAS_Evidence.red_hollow_cube,
	        RAS_Evidence.red_ball, RAS_Evidence.blue_cube, RAS_Evidence.blue_triangle,
	        RAS_Evidence.purple_cross, RAS_Evidence.purple_star, RAS_Evidence.an_object]

	def add_new_target_point(self, tar_data):
		if self.counter % self.takeEveryVal == 0:
	    		self.ls.append([tar_data.point.point.x, tar_data.point.point.y, tar_data.clas, tar_data.conf]) #z gives orientation
	    		self.process_list_update()
		self.counter += 1

	def process_list_update(self):
		#print(self.ls)
		if (len(self.ls) > 0):
			#Do clustering
			data = []
			for item in self.ls:
				data.append(item[0:2])
			clusters = hcluster.fclusterdata(data, self.threshold, criterion="distance")
			self.cluster_points = range(max(clusters))
			for c in range(max(clusters)):
				temp = [x == c+1 for x in clusters]
				self.cluster_points[c] = [item for item, con in zip(self.ls, temp) if con == True]
				self.check_for_new_targets(self.cluster_points[c]) # use cluster ids as object ids
			#print(self.cluster_points)
			#plt.scatter(*np.transpose(data), c=clusters)
			#plt.show()
				print("Finished Clustering")

	def most_common(self, L):
		groups = itertools.groupby(sorted(L))
		def _auxfun((item, iterable)):
			return len(list(iterable)), -L.index(item)
		return max(groups, key=_auxfun)[0]

	def check_for_new_targets(self, ls):
		#print("List: " + str(ls))
		if len(ls) < 2:
			print("Not enough data - waiting for new cluster.")
			return
		x_ls = []
		y_ls = []
		clas_ls = []
		conf_ls = []
		for pos in ls:
			x_ls.append(pos[0])
			y_ls.append(pos[1])
			clas_ls.append(pos[2])
			conf_ls.append(pos[3])
		x_avg = sum(x_ls)/len(x_ls)
		y_avg = sum(y_ls)/len(y_ls)

		#Determine clas
		#simple version
		#clas_max = self.most_common(clas_ls)
		#conf_avg = sum(conf_ls)/len(conf_ls)

		#complex version
		mbi = MakeTheBestOfIt(clas_ls, conf_ls)
		clas_max, conf_avg = mbi.getClassificationResult()

		print(clas_ls)
		print(str(clas_max) + " - " + self.classToName[clas_max-1])
		print(conf_ls)
		print(conf_avg)
		new_target = True
		update_target = False
		updated_id = 0
		if len(self.targets) > 0:
			for b in self.targets:
				if(self.calc_distance(b, x_avg, y_avg) < 0.05): # this value needs to be tested...
					new_target = False
					if b.clas != clas_max: #if at the same position the class of an object has changed, it should update map.
						new_target = False
						update_target = True
						updated_id = b.id
		else:
			new_target = True

		if new_target == True or update_target == True:
			if new_target:
				new_t = target(x_avg, y_avg, clas_max, conf_avg, self.currentIDCounter)
				print("Found new target object. Sending notification to Map.")
				self.targets.append(new_t)
				self.currentIDCounter += 1
			if update_target:
				new_t = target(x_avg, y_avg, clas_max, conf_avg, updated_id)
				print("Updated an object. Notification is sent to the map.")
				self.targets[updated_id] = new_t

			print("Found new target object. Sending notification to Map.")
			#self.targets.append(new_t)
			msg = ObjectMasterMsg()
			msg.point.point.x = new_t.x
			msg.point.point.y = new_t.y
			#New update target saftez strategz
			if update_target:
				msg.point.point.z = new_t.id + 100 #z coord used for storing the id.
			else:
				msg.point.point.z = new_t.id
			msg.clas = new_t.clas
			msg.conf = new_t.conf
			self.target_pub.publish(msg)

	def calc_distance(self, obj, x, y):
		return ((obj.x-x)**2.0+(obj.y-y)**2.0)**0.5


class battery_handler:
	def __init__(self):
		self.ls = []
		self.cluster_points = []

		self.id = 0
		self.threshold = 0.1

		self.batteries = []
		self.battery_pub = rospy.Publisher("/Geometry/New_Point", Point, queue_size=1)

		self.takeEveryVal = 1
		self.counter = 0

	def add_new_battery_point(self, bat_point):
		if self.counter % self.takeEveryVal == 0:
	    		self.ls.append([bat_point.x, bat_point.y, bat_point.z]) #z gives orientation
	    		self.process_list_update()
		self.counter += 1

	def process_list_update(self):
		#print(self.ls)
		if (len(self.ls) > 3):
			#Do clustering
			data = []
			for item in self.ls:
				data.append(item[0:2])
			clusters = hcluster.fclusterdata(data, self.threshold, criterion="distance")
			self.cluster_points = range(max(clusters))
			for c in range(max(clusters)):
				temp = [x == c+1 for x in clusters]
				self.cluster_points[c] = [item for item, con in zip(self.ls, temp) if con == True]
				self.check_for_new_batteries(self.cluster_points[c])
			#print(self.cluster_points)
			#plt.scatter(*np.transpose(data), c=clusters)
			#plt.show()
				print("Finished Clustering")

	def check_for_new_batteries(self, ls):
		#print("List: " + str(ls))
		if len(ls) < 3:
			print("Not enough data - waiting for new cluster.")
			return
		x_ls = []
		y_ls = []
		pose_ls = []
		for pos in ls:
			x_ls.append(pos[0])
			y_ls.append(pos[1])
			pose_ls.append(pos[2])

		x_avg = sum(x_ls)/len(x_ls)
		y_avg = sum(y_ls)/len(y_ls)
		pose_avg = sum(pose_ls)/len(pose_ls)
		if pose_avg < 1.5:
			pose_avg = 1
		else:
			pose_avg = 2

		new_battery = True
		if len(self.batteries) > 0:
			for b in self.batteries:
				if(self.calc_distance(b, x_avg, y_avg) < 0.1):
					new_battery = False
		else:
			new_battery = True

		if new_battery == True:
			new_b = battery(x_avg, y_avg, pose_avg)
			print("Found new battery. Sending notification to Map.")
			self.batteries.append(new_b)
			msg = Point()
			msg.x = new_b.x
			msg.y = new_b.y
			msg.z = new_b.pose
			self.battery_pub.publish(msg)

	def calc_distance(self, battery, x, y):
		return ((battery.x-x)**2.0+(battery.y-y)**2.0)**0.5

class object_handler:
	def __init__(self):

		self.target_sub = rospy.Subscriber("/object_master_out", ObjectMasterMsg, self.target_callback)
		self.battery_sub = rospy.Subscriber("battery/Geometry/New_Point", Point, self.battery_callback)
		self.bat_h = battery_handler()
		self.tar_h = target_handler()

	def battery_callback(self, data):
		self.bat_h.add_new_battery_point(data)

	def target_callback(self, data):
		self.tar_h.add_new_target_point(data)

#Overall class that organizes the other classes and the main node
class object_decision_master:
	def __init__(self):
		#setting variables
		rospy.init_node('object_decision_master', anonymous=True)
		#self.r = rospy.Rate(10)
		self.object_handler = object_handler()

	def make_decisions(self):
		try:
			rospy.spin()
		except KeyboardInterrupt:
			rospy.loginfo("Shutting down.")

if __name__=='__main__':
	obj_dec = object_decision_master()
	obj_dec.make_decisions()
