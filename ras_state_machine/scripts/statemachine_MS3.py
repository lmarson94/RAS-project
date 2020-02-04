#!/usr/bin/env python
import numpy as np
import rospy

###################
#   Import msgs   #
###################
# from geometry_msgs import PointStamped, Point
from ras_object_master.msg import ObjectMasterMsg
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
from ras_msgs.msg import RAS_Evidence

####################
#   Import srvs    #
####################
from arduino_servo_control.srv import *
from ras_gripper_controller.srv import *
from actionlib import *
import actionlib
import pure_pursuit_controller.msg
import os


class StateMachine(object):
    def __init__(self):
        self.node_name = "ras_state_machine"
        self.path_received = False
        self.has_target_obj = False
        self.object_detected = False
        self.obj_picked_up = False
        self.final_goal = None
        self.objects = []

        # States
        self.WAITFORGOAL = 1
        self.PLANNING = 2
        self.EXECUTION = 3
        self.CLASSIFICATION = 4     # Not used right now
        self.PICKUP = 5
        self.ERROR = 6
        self.EXPLORE = 7

        # Subscribers
        self.object_detection_sub = rospy.Subscriber("/object_master_out", ObjectMasterMsg, self.callback_object_detection, queue_size = 1)
        self.path_sub = rospy.Subscriber("/aPath", Path, self.path_callback, queue_size = 1)
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback_goal, queue_size = 1)

        # Wait for service providers
        rospy.loginfo("Waiting for gripper server..")
        rospy.wait_for_service("/gripper_command")
        rospy.loginfo("Connected gripper server!")

        # Publishers
        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

    	# Set up action clients
        rospy.loginfo("%s: Waiting for action server...", self.node_name)
        self.move_robot_ac = SimpleActionClient("/purePursuit", pure_pursuit_controller.msg.PurePursuitAction)
        if not self.move_robot_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to action server", self.node_name)

        # Init state machine
        self.state = self.WAITFORGOAL
        rospy.sleep(3)
        self.check_states()

    def say(self, word):
        os.system("~/catkin_ws/src/ras_project_group1/audio_common/sound_play/scripts/say.py " + word)

    def getMostValuableObject():
        maxObj = self.objects[0]
        for obj in self.objects:
            if obj.score > maxObj.score:
                maxObj = obj
        return maxObj


    def check_states(self):
        while not rospy.is_shutdown() and self.state != self.ERROR:

            if self.state == self.WAITFORGOAL:
                if self.final_goal is not None:
                    self.state = self.PLANNING

            elif self.state == self.PLANNING:
                #rospy.loginfo("PLANNING!")
                if self.path_received:
                    rospy.loginfo("Path received")
                    self.say("Planning")
                    goal = pure_pursuit_controller.msg.PurePursuitGoal()
                    goal.intermediatePoints.poses = self.path.poses
                    self.move_robot_ac.send_goal(goal)
                    self.path_received = False
                    self.state = self.EXECUTION

            elif self.state == self.EXPLORE:
                if self.move_robot_ac.get_state() == GoalStatus.SUCCEEDED:
                    if not self.has_target_obj:
                        self.create_TargetPose_msg(getMostValuableObject().pose)
                        self.goal_pub.publish(obj)
                        self.state = self.PLANNING
                        self.has_target_obj = True
                    elif not self.obj_picked_up:
                        self.state = self.PICKUP
                    else:
                        self.state = self.WAITFORGOAL

            elif self.state == self.EXECUTION:
                #rospy.loginfo("EXECUTING!");
                if self.object_detected and not self.has_target_obj:
                    rospy.loginfo("Got in here at least")
                    self.object_detected = False
                    self.has_target_obj = True
                    self.move_robot_ac.cancel_goal()
                    obj = self.create_TargetPose_msg(self.objects[0].pose)
                    rospy.loginfo("Replanning")
                    self.goal_pub.publish(obj)
                    self.state = self.PLANNING

                else:
                    if self.move_robot_ac.get_state() == GoalStatus.SUCCEEDED:
                        if self.obj_picked_up:
                            rospy.loginfo("REACHED FINAL GOAL WITH OBJECT!!!")
                            self.state = self.WAITFORGOAL
                        elif not self.objects:
                            rospy.loginfo("REACHED FINAL GOAL WITHOUT OBJECT!!!")
                            self.state = self.WAITFORGOAL
                        else:
                            self.state = self.PICKUP

            elif self.state == self.PICKUP:
                gripper_srv = rospy.ServiceProxy('/gripper_command', GripperCommand)
                rospy.loginfo("Grabbing object...")
                #gripper_req = gripper_srv("up_open")
                rospy.sleep(1)
                gripper_req = gripper_srv("down_open")
                rospy.sleep(1)
                gripper_req = gripper_srv("down_close")
                rospy.sleep(1)
                gripper_req = gripper_srv("up_close")
                rospy.sleep(1)
                rospy.loginfo("Object grabbed!")
                self.obj_picked_up = True

                self.goal_pub.publish(self.final_goal)
                self.state = self.PLANNING

            # Error handling
            elif self.state == self.ERROR:
                rospy.logerr("%s: State machine failed", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

    def callback_object_detection(self, msg):
        if not msg.point.point.x == -99 and not msg.clas == 15:  # TODO: Remove last condition
          self.object_detected = True
          obj = Object(msg.clas, msg.point.point, msg.conf)
          rospy.loginfo("Object pose: %f" % msg.point.point.x)
          self.objects.append(obj)
          rospy.loginfo("Number of objects found : " + str(len(self.objects)))
          rospy.loginfo("Object class: " + str(obj.clas))
          rospy.loginfo("Object name: " + obj.name)
          self.say("\"Detected " + str(obj.name) + "\"")
          gripper_srv = rospy.ServiceProxy('/gripper_command', GripperCommand)
          rospy.loginfo("Opening gripper")
          gripper_req = gripper_srv("down_open")

        else:
          #self.say("\"Detected an objec with class " + str(msg.clas) + " and x:" + str(msg.point.point.x) +"\"")
          pass

    def callback_obstacle_detection(self, msg):
        self.obstacle_detected = True
        self.say("obstacle detected, must abort")

    def create_TargetPose_msg(self, point):
       pose = PoseStamped()
       pose.pose.position.x = point.x
       pose.pose.position.y = point.y
       pose.pose.position.z = point.z

       return pose

    def path_callback(self , msg):
        rospy.loginfo("Received path from planner!")
        self.path_received = True
        self.path = msg

    def callback_goal(self, msg):
        if self.final_goal is None:
            rospy.loginfo("Set final goal")
            self.final_goal = msg

class Object(object):
    # {ras_msgs::RAS_Evidence::yellow_ball, ras_msgs::RAS_Evidence::yellow_cube, ras_msgs::RAS_Evidence::green_cube, ras_msgs::RAS_Evidence::green_cylinder,
    #  ras_msgs::RAS_Evidence::green_hollow_cube, ras_msgs::RAS_Evidence::orange_cross, ras_msgs::RAS_Evidence::patric, ras_msgs::RAS_Evidence::red_cylinder,
    #  ras_msgs::RAS_Evidence::red_hollow_cube, ras_msgs::RAS_Evidence::red_ball, ras_msgs::RAS_Evidence::blue_cube, ras_msgs::RAS_Evidence::blue_triangle,
    #  ras_msgs::RAS_Evidence::purple_cross, ras_msgs::RAS_Evidence::purple_star};
    classToName = [RAS_Evidence.yellow_ball, RAS_Evidence.yellow_cube,
        RAS_Evidence.green_cube, RAS_Evidence.green_cylinder,
        RAS_Evidence.green_hollow_cube, RAS_Evidence.orange_cross,
        RAS_Evidence.patric, RAS_Evidence.red_cylinder, RAS_Evidence.red_hollow_cube,
        RAS_Evidence.red_ball, RAS_Evidence.blue_cube, RAS_Evidence.blue_triangle,
        RAS_Evidence.purple_cross, RAS_Evidence.purple_star, RAS_Evidence.an_object]

    nameToScore = {
        RAS_Evidence.yellow_ball : 1,
        RAS_Evidence.yellow_cube : 1,
        RAS_Evidence.green_cube : 1,
        RAS_Evidence.green_cylinder : 1,
        RAS_Evidence.green_hollow_cube : 1,
        RAS_Evidence.orange_cross : 1,
        RAS_Evidence.patric : 1,
        RAS_Evidence.red_cylinder : 1,
        RAS_Evidence.red_hollow_cube : 1,
        RAS_Evidence.red_ball : 1,
        RAS_Evidence.blue_cube : 1,
        RAS_Evidence.blue_triangle : 1,
        RAS_Evidence.purple_cross : 1,
        RAS_Evidence.purple_star : 1,
        RAS_Evidence.an_object : 0
    }

    def __init__(self, clas, point, conf):
        self.clas = clas
        self.pose = point
        self.conf = conf
        self.name = self.classToName[clas-1]
        self.score = nameToScore[self.name]

if __name__ == "__main__":

    rospy.init_node('ras_state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
