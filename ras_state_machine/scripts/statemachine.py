#!/usr/bin/env python
import numpy as np
import rospy

###################
#   Import msgs   #
###################
# from geometry_msgs import PointStamped, Point
from object_detection.msg import msg_objects_detected
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from std_msgs.msg import String

####################
#   Import srvs    #
####################
from arduino_servo_control.srv import *
from ras_gripper_controller.srv import *
from actionlib import *
import actionlib
import pure_pursuit_controller.msg


class StateMachine(object):
    def __init__(self):
        self.node_name = "ras_state_machine"

        # Subscribers
        self.object_detection_sub = rospy.Subscriber("/objects_detected", msg_objects_detected, self.callback_object_detection)
        self.pose_est_sub = rospy.Subscriber("/position", PointStamped, self.callback_pose_estimate)
        self.obstacle_detection_sub = rospy.Subscriber("/obstacle_detected", String, self.callback_obstacle_detection)

        # Wait for service providers
        rospy.loginfo("Waiting for gripper server..")
        rospy.wait_for_service("/gripper_command")
        rospy.loginfo("Connected gripper server!")
        # Instantiate publishers
        #self.target_pose_pub = rospy.Publisher("/target_pose", PointStamped, queue_size=10)

        rospy.loginfo("IN STATE MACHINE")

	# Set up action clients
        rospy.loginfo("%s: Waiting for action server...", self.node_name)
        self.move_robot_ac = SimpleActionClient("/purePursuit", pure_pursuit_controller.msg.PurePursuitAction)
        if not self.move_robot_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to action server", self.node_name)

	self.should_go_to_obj = rospy.get_param("should_go_to_obj")
	self.x_target = rospy.get_param("x_position")
	self.y_target = rospy.get_param("y_position")


        # Init state machine
        self.state = 1
        rospy.sleep(3)
        self.check_states()


    def check_states(self):
        while not rospy.is_shutdown() and self.state != 4:

            # State 1:  Detect Object
            if self.state == 1:

		if self.should_go_to_obj:
		    self.state = 2
		    continue

                rospy.loginfo("%s: Detecting object...", self.node_name)
                self.object_detected = False
                rospy.sleep(1)

                if self.object_detected:
                    rospy.loginfo("Object detected!")

                    self.pose_estimate = False
                    rospy.loginfo("%s: Estimating pose of object...", self.node_name)
                    rospy.sleep(1)

                    if self.pose_estimate:
                        rospy.loginfo("Distance to object estimated!")
                        # rate = rospy.Rate(5)
                        # while not rospy.is_shutdown():
                        #     #self.target_pose_pub.publish(self.object)
                        #     rate.sleep()
                        self.state = 2
                    else:
                        rospy.loginfo("Failed to estimate distance to object!")
                        self.state = 5
                else:
                    rospy.loginfo("Object not detected, abort mission!")
                    self.state = 5

            # State 2: Move to object
            if self.state == 2:
                goal = pure_pursuit_controller.msg.PurePursuitGoal()

                pose1 = Pose()

		if self.should_go_to_obj:
		    pose1.position.x = self.x_target
		    pose1.position.y = self.y_target
		else:
                    pose1.position.x = self.object.point.x
                    pose1.position.y = self.object.point.y
                poses = []
                poses.append(pose1)
                goal.intermediatePoints = poses
                self.move_robot_ac.send_goal(goal)

                rospy.loginfo("Moving towards object")
                self.obstacle_detected = False
                self.keep_looking = True

                while self.keep_looking:
                    if self.obstacle_detected:
                        self.move_robot_ac.cancel_goal()
                        rospy.loginfo("Obstacle detected, aborting mission!")
                        self.keep_looking = False
                        self.state = 5
                    elif self.move_robot_ac.get_state() == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Reached the goal pose!")
                        self.keep_looking = False
                        self.state = 3

            # State 3: Grab object
            ##########################################
            # Services:                              #
            #   up_close                             #
            #   up_open                              #
            #   down_close                           #
            #   down_open                            #
            # Default: up_close                      #
            ##########################################

            if self.state == 3:

                gripper_srv = rospy.ServiceProxy('/gripper_command', GripperCommand)
                rospy.loginfo("Grabbing object...")
                gripper_req = gripper_srv("up_open")
                rospy.sleep(1)
                gripper_req = gripper_srv("down_open")
                rospy.sleep(1)
                gripper_req = gripper_srv("down_close")
                rospy.sleep(1)
                gripper_req = gripper_srv("up_close")
                rospy.sleep(1)
                
                rospy.loginfo("Object grabbed!")
                self.state = 4

            # State 4: Move to target pose with object
            if self.state == 4:
                goal = pure_pursuit_controller.msg.PurePursuitGoal()

                pose2 = Pose()
                pose2.position.x = 0.207
                pose2.position.y = 0.202
                poses = []
                poses.append(pose2)
                goal.intermediatePoints = poses
                self.move_robot_ac.send_goal(goal)

                rospy.loginfo("Moving towards object")
                self.obstacle_detected = False
                self.keep_looking = True

                while self.keep_looking:
                    if self.obstacle_detected:
                        self.move_robot_ac.cancel_goal()
                        rospy.loginfo("Obstacle detected, aborting mission!")
                        self.keep_looking = False
                        self.state = 5
                    elif self.move_robot_ac.get_state() == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Reached the goal pose!")
                        self.keep_looking = False
                        self.state = 99

            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

    def callback_object_detection(self, msg):
        self.object_detected = True

    def callback_pose_estimate(self, msg):
        self.pose_estimate = True
        self.object = msg

    def callback_obstacle_detection(self, msg):
        self.obstacle_detected = True

    def create_TargetPose_msg(self, x, y, z):
        p = PointStamped()
        p.position.x = x
        p.position.y = y
        p.position.z = z

        return p

if __name__ == "__main__":

    rospy.init_node('ras_state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
