#!/usr/bin/env python

import rospy
from arduino_servo_control.srv import *
from ras_gripper_controller.srv import *
from std_msgs.msg import String

class GripperController:
	def __init__(self):
		self.pub = rospy.Publisher("gripper/feedback", String, queue_size = 1)
		self.service = rospy.Service('gripper_command', GripperCommand, self.gripper_command_service)
		
	def call_service(self, servo_lift, servo_grab):
		rospy.wait_for_service('/arduino_servo_control/set_servo_angles')
		success = String()
		#response = SetServoAnglesResponse()
		success.data = "Operation Fail"
		try:
			set_servo_angles = rospy.ServiceProxy('/arduino_servo_control/set_servo_angles', SetServoAngles)
			resp = set_servo_angles(servo_grab, servo_lift)
			if (resp.success == True):
				success.data = "Operation Success"
		except rospy.ServiceException as e:
			print("Service call failed: %s", e)

		self.pub.publish(success)
		return resp.success

	def gripper_command_service(self, req):
		rospy.loginfo("Received command %s", req.gripper_command)
		if (req.gripper_command == "up_close"):
			return self.call_service(110, 0)
		elif (req.gripper_command == "up_open"):
			return self.call_service(110, 90)
		elif (req.gripper_command == "down_close"):
			return self.call_service(30, 0)
		elif (req.gripper_command == "down_open"):
			return self.call_service(30, 90)
		else:
			error_msg = String()
			error_msg.data = "Unknown Command"
			self.pub.publish(error_msg)
			return False


	def callback_cmd(self, command):
		if (command.data == "up_close"):
			self.call_service(110, 0)
		elif (command.data == "up_open"):
			self.call_service(110, 90)
		elif (command.data == "down_close"):
			self.call_service(30, 0)
		elif (command.data == "down_open"):
			self.call_service(30, 90)
		else:
			error_msg = String()
			error_msg.data = "Unknown Command"
			self.pub.publish(error_msg)

	def gripperControl(self):
		rospy.init_node('gripper_control')
		rospy.Subscriber('gripper/cmd', String, self.callback_cmd)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()

if __name__ == '__main__':
	controller = GripperController()
	controller.gripperControl()
