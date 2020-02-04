#!/usr/bin/env python
import numpy as np
import rospy
from enum import Enum
from persistant_storage import *

###################
#   Import msgs   #
###################
# from geometry_msgs import PointStamped, Point
from ras_object_master.msg import ObjectMasterMsg
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped, Point, Twist
from std_msgs.msg import String
from nav_msgs.msg import Path, OccupancyGrid
from ras_msgs.msg import RAS_Evidence
from occupancy_grid.msg import points_msg, GridMap

####################
#   Import srvs    #
####################
from arduino_servo_control.srv import *
from ras_gripper_controller.srv import *
from impossible_object_pickup.srv import *
from actionlib import *
import actionlib
import pure_pursuit_controller.msg
import map_explorer.msg
import os

class State(Enum):
    WAITFORGOAL = 1
    PLANNING = 2
    EXECUTION = 3
    CLASSIFICATION = 4
    PICKUP = 5
    ERROR = 6
    EXPLORE = 7
    RESCUE = 8
    TEST = 9
    FINISHED = 10

class StateMachine():
    def __init__(self, initial_state):
        self.node_name = "ras_state_machine"
        self.path_received = False
        self.has_target_obj = False
        self.hasReceivedMap = False
        self.hasReceivedFeedbackMap = False
        self.isCurrentlyExploring = False
        self.object_detected = False
        self.obj_picked_up = False
        self.final_goal = None
        self.objects = []
    	self.new_wall = False
    	self.new_battery = False
        self.obstacle_detected = False
        self.closeToGoal = False
        self.point2 = None
        self.occupancy_grid_og = map_explorer.msg.ExplorationGoal()
        self.exploration_grid = map_explorer.msg.ExplorationGoal()

        if initial_state == State.RESCUE:
            rospy.loginfo("Loading seen objects")
            self.initiate_from_memory()
            rospy.loginfo("LOADED OBJECTS")

        # Subscribers
        self.object_detection_sub = rospy.Subscriber("/target/Geometry/New_Point", ObjectMasterMsg, self.callback_object_detection, queue_size = 1)
        self.path_sub = rospy.Subscriber("/aPath", Path, self.path_callback, queue_size = 1)
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback_goal, queue_size = 1)
        self.new_wall_sub = rospy.Subscriber("/Mapping/points", points_msg, self.callback_wall, queue_size = 1)
        self.battery_sub = rospy.Subscriber("/Geometry/New_Point", Point, self.callback_battery, queue_size = 1)
        self.obstacle_detection_sub = rospy.Subscriber("/obstacle_detected", String, self.callback_obstacle_detection, queue_size = 1)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.callback_map, queue_size = 1)
        # Wait for service providers
        rospy.loginfo("Waiting for gripper server..")
        rospy.wait_for_service("/gripper_command")
        rospy.loginfo("Connected gripper server!")
        rospy.loginfo("Waiting for pickup server..")
        rospy.wait_for_service("pickup_impossible_object");
        rospy.loginfo("Connected to pickup server!")

        # Publishers
        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_speed = rospy.Publisher("/motor_controller/twist", Twist, queue_size=1)

    	# Set up action clients
        rospy.loginfo("%s: Waiting for action server...", self.node_name)
        self.move_robot_ac = SimpleActionClient("/purePursuit", pure_pursuit_controller.msg.PurePursuitAction)
        self.exploration_ac = SimpleActionClient("map_explorer", map_explorer.msg.ExplorationAction)
        if not self.move_robot_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to pure pursuit action server", self.node_name)
            exit()
        if not self.exploration_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to exploration action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to action server", self.node_name)

        # Init state machine
        self.state = initial_state
        rospy.sleep(3)
        self.check_states()

    def say(self, word):
        os.system("~/catkin_ws/src/ras_project_group1/audio_common/sound_play/scripts/say.py " + word)

    def check_states(self):
        while not rospy.is_shutdown() and self.state != State.ERROR:
            rospy.sleep(0.05)

            if self.state == State.WAITFORGOAL:
                self.waitforgoal()
            elif self.state == State.PLANNING:
                self.planning()

            elif self.state == State.EXECUTION:
                self.execution_new()

            elif self.state == State.PICKUP:
                self.pickup()

            elif self.state == State.EXPLORE:
                self.explore()

            elif self.state == State.RESCUE:
                self.rescue()

            elif self.state == State.TEST:
                self.testing()

            elif self.state == State.FINISHED:
                rospy.loginfo("%s: State machine finished!", self.node_name)
                exit()

    def get_most_valuable_object(self):
        maxObj = self.objects[0]
        for obj in self.objects:
            if obj.score > maxObj.score:
                maxObj = obj
        return maxObj

    def has_seen_object(self, object):
        for obj in self.objects:
            if obj.id == object.id and obj.clas == object.clas:
                return True
        return False

    def initiate_from_memory(self):
        for obj in read_saved_objects():
            rospy.loginfo("Read object " + str(obj))
            self.objects.append(obj)

    def testing(self):
        if self.new_battery:
            self.new_battery = False
            rospy.loginfo("Battery detected!")
        elif self.obstacle_detected:
            self.obstacle_detected = False
            rospy.loginfo("Obstacle detected!")


    def waitforgoal(self):
        if self.final_goal is not None:
            self.state = State.PLANNING

    def planning(self):
        #rospy.loginfo("PLANNING!")
        if self.path_received:
            rospy.loginfo("Path received")
            self.say("Planning")
            tempPose = PoseStamped()
            if self.point2 is not None:
                tempPose.pose.position.x = self.point2.x
                tempPose.pose.position.y = self.point2.y
                self.path.poses.append(tempPose)
            goal = pure_pursuit_controller.msg.PurePursuitGoal()
            goal.intermediatePoints.poses = self.path.poses
            self.say("Start")
            self.move_robot_ac.send_goal(goal,
                                         feedback_cb=self.callback_purepursuit_fb)
            self.path_received = False
            self.state = State.EXECUTION

    def rescue(self):
        rospy.loginfo("length of objects: %d" % len(self.objects))
        if len(self.objects) > 0:
            most_valuable_obj = self.get_most_valuable_object()
            self.say("\"Going to " + str(most_valuable_obj.name) + "\"")
            if self.isInForbiddenArea(most_valuable_obj):
                rospy.loginfo("Picking up forbidden object")
                self.objects.pop(self.objects.index(most_valuable_obj))
                pickup_srv = rospy.ServiceProxy("/pickup_impossible_object", PickupImpossibleObject);
                try:
                    pickup_response = pickup_srv(most_valuable_obj.pose)
                except rospy.ServiceException as exc:
                    print("Forbidden object uncreachable: " + str(exc))
                    return
                self.final_goal = self.create_target_pose_msg(pickup_response.point1)
                self.point2 = pickup_response.point2
                self.goal_pub.publish(self.final_goal)
                self.has_target_obj = True
                self.obj_picked_up = False
                self.state = State.PLANNING
            else:
                self.objects.pop(self.objects.index(most_valuable_obj))
                self.final_goal = self.create_target_pose_msg(most_valuable_obj.pose)
                self.goal_pub.publish(self.final_goal)
                self.has_target_obj = True
                self.obj_picked_up = False
                self.state = State.PLANNING
        else:
            self.state == State.FINISHED


    def isInForbiddenArea(self,object):
        # return True;
        resolution = self.occupancy_grid_og.gridmap.map.info.resolution
        map_x = int(object.pose.x/resolution)
        map_y = int(object.pose.y/resolution)
        index = int(map_y*self.occupancy_grid_og.gridmap.map.info.width+map_x)
        if self.occupancy_grid_og.gridmap.map.data[index] > 25:
            return True
        else:
            return False

    def execution_new(self):
        if self.obstacle_detected:
            self.obstacle_detection()

        if self.new_battery:
            self.battery_detected()

        if self.object_detected and not self.has_target_obj and self.closeToGoal:
            self.closeToGoal = False
            self.object_detected = False
            self.has_target_obj = True
            self.move_robot_ac.cancel_goal()
            self.final_goal = self.create_target_pose_msg(self.target_obj.pose)
            rospy.loginfo("Replanning")
            self.goal_pub.publish(self.final_goal)
            gripper_srv = rospy.ServiceProxy('/gripper_command', GripperCommand)
            rospy.loginfo("Opening gripper")
            gripper_req = gripper_srv("down_open")
            self.state = State.PLANNING

        if self.move_robot_ac.get_state() == GoalStatus.SUCCEEDED:
            if self.obj_picked_up:
                rospy.loginfo("Reached final goal with object")
                gripper_srv = rospy.ServiceProxy('/gripper_command', GripperCommand)
                rospy.loginfo("Releasing object...")
                gripper_req = gripper_srv("down_open")
                rospy.sleep(1)
                gripper_req = gripper_srv("up_close")
                rospy.sleep(1)
                self.state = State.RESCUE
            else:
                self.state = State.PICKUP

    #Old execution state
    def execution(self):
        #rospy.loginfo("EXECUTING!");
        if self.obstacle_detected:
            self.obstacle_detection()

        elif self.object_detected and not self.has_target_obj:
            self.object_detected = False
            self.has_target_obj = True
            self.move_robot_ac.cancel_goal()
            self.obj = self.create_target_pose_msg(self.objects[-1].pose)
            rospy.loginfo("Replanning")
            self.goal_pub.publish(self.obj)
            self.state = State.PLANNING

        elif self.new_battery:
            self.battery_detected()

        # if self.new_wall and not self.state == self.PICKUP:
        #     rospy.loginfo("New wall detected, REPLAN!")
        #     self.new_wall = False
        #     self.move_robot_ac.cancel_goal()
        #     self.goal_pub.publish(self.final_goal)
        #     self.state = self.PLANNING

        else:
            if self.move_robot_ac.get_state() == GoalStatus.SUCCEEDED:
                if self.obj_picked_up:
                    rospy.loginfo("Reached final goal with object")
                    gripper_srv = rospy.ServiceProxy('/gripper_command', GripperCommand)
                    rospy.loginfo("Releasing object...")
                    gripper_req = gripper_srv("down_open")
                    rospy.sleep(1)
                    gripper_req = gripper_srv("up_close")
                    rospy.sleep(1)
                    self.state = State.RESCUE
                elif not self.objects:
                    rospy.loginfo("Reached final goal WITHOUT object")
                    self.state = State.RESCUE
                else:
                    self.state = State.PICKUP

    def battery_detected(self):
        if self.state == State.EXPLORE:
            self.exploration_ac.cancel_goal()
            rospy.loginfo("New battery detected, REPLAN!")
            rospy.sleep(0.1)
        elif self.state == State.EXECUTION:
            print("Battery handled in Execution")
            self.move_robot_ac.cancel_goal()
            self.new_battery = False
            self.goal_pub.publish(self.final_goal)
            self.state = State.PLANNING

    def obstacle_detection(self):
        if self.state == State.EXPLORE:
            self.exploration_ac.cancel_goal()
        elif self.state == State.EXECUTION:
            self.move_robot_ac.cancel_goal()

        rospy.sleep(0.1)
        rescue = Twist()
        rescue.linear.x = -0.1
        rescue.linear.y = -0.1
        count = 0
        rospy.loginfo("Obstacle detected, going backwards")
        # rospy.loginfo("count BEFORE: %d", count)
        # while count < 1700:
        while count < 50:
            # rospy.loginfo("count: %d", count)
            self.pub_speed.publish(rescue)
            rospy.sleep(0.05)
            count += 1
        print("OBSTACLE DETECTION DONE!")
        self.obstacle_detected = False

        if not self.state == State.EXPLORE:
            rospy.loginfo("REPLAN!")
            self.goal_pub.publish(self.final_goal)
            self.state = State.PLANNING

    def pickup(self):
        self.point2 = None
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

        start = Point()
        start.x = 0.25
        start.y = 0.4
        start_pose = self.create_target_pose_msg(start)
        self.final_goal = start_pose
        self.goal_pub.publish(self.final_goal)
        self.state = State.PLANNING

    def explore(self):
        if self.hasReceivedMap and not self.isCurrentlyExploring:
            rospy.loginfo("Starting exploration")
            self.exploration_ac.send_goal(self.occupancy_grid_og,
                                          feedback_cb=self.callback_fb)
            self.isCurrentlyExploring = True

        elif self.isCurrentlyExploring:
            rospy.sleep(0.1)
            if self.obstacle_detected:
                self.obstacle_detection()
                self.sendNewMap()

            if self.object_detected:
                self.exploration_ac.cancel_goal()
                self.sendNewMap()
                self.object_detected = False
                print("Updated mAP!!!!!!!!!!!!!!!!!")

            if self.new_battery:
                self.battery_detected()
                print("Battery detected function done")
                self.sendNewMap()
                print("Sending new map")
                self.new_battery = False

            if self.exploration_ac.get_state() == GoalStatus.SUCCEEDED:
                self.isCurrentlyExploring = False
                # TODO: FIGURE OUT AND FINISH!
                self.state = State.FINISHED

    def updateExploreMap(self):
        indices_100 = [i for i, x in enumerate(self.occupancy_grid_og.gridmap.map.data) if x == 100] # Get indices of occupied cells
        indices_75 = [i for i, x in enumerate(self.occupancy_grid_og.gridmap.map.data) if x == 75]   # Get indices of dark gray (forbidden)
        indices_50 = [i for i, x in enumerate(self.occupancy_grid_og.gridmap.map.data) if x == 50]   # Get indices of gray (forbidden)

        for idx in indices_50:
            self.exploration_grid.gridmap.map.data[idx] == 50
        for idx in indices_75:
            self.exploration_grid.gridmap.map.data[idx] == 75
        for idx in indices_100:
            self.exploration_grid.gridmap.map.data[idx] == 100


    def sendNewMap(self):
        while not self.hasReceivedFeedbackMap:
            continue
        self.hasReceivedFeedbackMap = False
        self.updateExploreMap()
        with open('occupancy_grid.txt', 'w') as f:
            for item in self.occupancy_grid_og.gridmap.map.data:
                f.write("%s\n" % item)
        self.exploration_ac.send_goal(self.exploration_grid,
                                      feedback_cb=self.callback_fb)

    def error():
        rospy.logerr("%s: State machine failed", self.node_name)
        return

    def callback_object_detection(self, msg):
        if self.state == State.EXPLORE:
            if not msg.point.point.x == -99:
                if msg.clas > 99:
                    msg.clas = msg.clas - 100;
                obj = Object(msg.clas, msg.point.point, msg.conf)
                if not self.has_seen_object(obj):
                    rospy.loginfo("Saving object to file")
                    write_new_object(obj)

        if not msg.point.point.x == -99 and not msg.clas > 99:  # TODO: Remove last condition
          if self.state == State.EXPLORE:
              self.object_detected = True
              obj = Object(msg.clas, msg.point.point, msg.conf)
              rospy.loginfo("Object pose: %f" % msg.point.point.x)
              self.objects.append(obj)
              rospy.loginfo("LENGTH OF THE LIST!!!! : " + str(len(self.objects)))
              rospy.loginfo("SOME CLASS: " + str(obj.clas))
              rospy.loginfo("Object name: " + obj.name)
              self.say("\"Detected " + str(obj.name) + "\"")
          else:
              self.object_detected = True
              obj = Object(msg.clas, msg.point.point, msg.conf)
              rospy.loginfo("Object pose: %f" % msg.point.point.x)
              self.target_obj = obj
              rospy.loginfo("SOME CLASS: " + str(self.target_obj.clas))
              rospy.loginfo("Object name: " + obj.name)
              self.say("\"Detected " + str(obj.name) + "\"")

        else:
          #self.say("\"Detected an objec with class " + str(msg.clas) + " and x:" + str(msg.point.point.x) +"\"")
          pass

    def callback_obstacle_detection(self, msg):
        self.obstacle_detected = True

    def create_target_pose_msg(self, point):
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

    def callback_wall(self, msg):
        self.new_wall = True

    def callback_battery(self, msg):
        print("Callback battery detected called.")
        self.new_battery = True

    def callback_map(self, msg):
        self.hasReceivedMap = True
        self.occupancy_grid_og.gridmap.map = msg
        self.occupancy_grid_og.gridmap.n_width = msg.info.width
        self.occupancy_grid_og.gridmap.n_height = msg.info.height
        self.occupancy_grid_og.gridmap.resolution = msg.info.resolution

    def callback_fb(self, fb):
        # print("Received feedback!!")
        self.hasReceivedFeedbackMap = True
        self.exploration_grid.gridmap.map = fb.feedback
        self.exploration_grid.gridmap.n_width = fb.feedback.info.width
        self.exploration_grid.gridmap.n_height = fb.feedback.info.height
        self.exploration_grid.gridmap.resolution = fb.feedback.info.resolution

    def callback_purepursuit_fb(self, fb):
        # print("Received feedback!!")
        self.closeToGoal = True
        if not self.obj_picked_up:
            gripper_srv = rospy.ServiceProxy('/gripper_command', GripperCommand)
            rospy.loginfo("Opening gripper - close to goal.")
            rospy.sleep(1)
            gripper_req = gripper_srv("down_open")


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
        RAS_Evidence.yellow_ball : 10000,
        RAS_Evidence.yellow_cube : 1000,
        RAS_Evidence.green_cube : 1000,
        RAS_Evidence.green_cylinder : 1000,
        RAS_Evidence.green_hollow_cube : 100,
        RAS_Evidence.orange_cross : 100,
        RAS_Evidence.patric : 5000,
        RAS_Evidence.red_cylinder : 1000,
        RAS_Evidence.red_hollow_cube : 100,
        RAS_Evidence.red_ball : 10000,
        RAS_Evidence.blue_cube : 1000,
        RAS_Evidence.blue_triangle : 5000,
        RAS_Evidence.purple_cross : 100,
        RAS_Evidence.purple_star : 5000,
        RAS_Evidence.an_object : 0
    }

    # nameToScore = {
    #     RAS_Evidence.yellow_ball : 10000,
    #     RAS_Evidence.yellow_cube : 1000,
    #     RAS_Evidence.green_cube : 1000,
    #     RAS_Evidence.green_cylinder : 7000,
    #     RAS_Evidence.green_hollow_cube : 7000,
    #     RAS_Evidence.orange_cross : 100,
    #     RAS_Evidence.patric : 200,
    #     RAS_Evidence.red_cylinder : 1000,
    #     RAS_Evidence.red_hollow_cube : 100,
    #     RAS_Evidence.red_ball : 1000,
    #     RAS_Evidence.blue_cube : 1000,
    #     RAS_Evidence.blue_triangle : 5000,
    #     RAS_Evidence.purple_cross : 10000,
    #     RAS_Evidence.purple_star : 10000,
    #     RAS_Evidence.an_object : 0
    # }

    def __init__(self, clas, point, conf):
        self.clas = clas
        self.pose = point
        self.conf = conf
        self.name = self.classToName[clas-1]
        self.score = self.nameToScore[self.name]
        self.id = point.z

def run_state_machine(state):
    if state == None:
        state = State.WAITFORGOAL
    try:
        StateMachine(state)
    except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node('ras_state_machine')
    if len(sys.argv) > 1:
        if sys.argv[1] == '-explore':
            run_state_machine(State.EXPLORE)
        elif sys.argv[1] == '-rescue':
            run_state_machine(State.RESCUE)      #Change to rescue
        elif sys.argv[1] == '-test':
            run_state_machine(State.TEST)
    else:
        run_state_machine(State.WAITFORGOAL)

    rospy.spin()
