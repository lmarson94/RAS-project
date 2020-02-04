#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pure_pursuit_controller/PurePursuitAction.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

void path_callback(const nav_msgs::Path::ConstPtr& msg)
{
  actionlib::SimpleActionClient<pure_pursuit_controller::PurePursuitAction> ac("purePursuit", true);
  ROS_INFO("Waiting for action server to start");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal");
  pure_pursuit_controller::PurePursuitGoal goal;
  goal.intermediatePoints.poses = msg->poses;
  ac.sendGoal(goal);

  bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(360.0));

  if (finishedBeforeTimeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before time out.");
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_pure_pursuit");
  ros::NodeHandle n;

  // actionlib::SimpleActionClient<pure_pursuit_controller::PurePursuitAction> ac("purePursuit", true);
  ros::Subscriber path_sub = n.subscribe<nav_msgs::Path>("/aPath", 1, path_callback);

  // ros::Duration(3).sleep();
  ros::spin();

  // ROS_INFO("Waiting for action server to start");
  // ac.waitForServer();
  //
  // ROS_INFO("Action server started, sending goal");
  // pure_pursuit_controller::PurePursuitGoal goal;
  // geometry_msgs::Pose pose0, pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9, pose10, pose11, poseN;
  // pose1.position.x = 0.202; pose1.position.y = 1.5;
  // pose2.position.x = 1; pose2.position.y = 2.25;
  // pose3.position.x = 2.25; pose3.position.y = 2.25;
  // pose4.position.x = 2.25; pose4.position.y = 0.207;
  // pose5.position.x = 0.65; pose5.position.y = 0.207;
  // pose6.position.x = 0.65; pose6.position.y = 0.85;
  // pose7.position.x = 1.6; pose7.position.y = 0.9;
  // pose8.position.x = 1.8; pose8.position.y = 2.25;
  // pose9 = pose2;
  // pose10 = pose1;
  // pose11.position.x = 0.202; pose11.position.y = 0.207;
  // pose0 = pose11;
  // poseN.position.x = 0.202; poseN.position.y = 0.207;
  // //ROS_INFO("%s", typeid(goal.intermediatePoints).name());
  // std::vector<geometry_msgs::PoseStamped> poses(4);
  // std::vector<geometry_msgs::Pose> backPose(1);
  // poses[0].pose = pose0; poses[1].pose = pose1; poses[2].pose = pose2; poses[3].pose = pose3; //poses[4].pose = pose4; poses[5].pose = pose5;
  // // poses[6].pose = pose6; poses[7].pose = pose7; poses[8].pose = pose8; poses[9].pose = pose9; poses[10].pose = pose10;
  // //poses[11].pose = pose11;
  // /*poses[0] = pose4; poses[1] = pose5; poses[2] = pose11;*/
  // /*poses[0] = pose1; poses[1] = pose11; poses[2] = pose1; poses[3] = pose11;*/
  // goal.intermediatePoints.poses = poses;
  // //goal.intermediatePoints[0] = pose1;
  // //goal.intermediatePoints[1] = pose3]2;
  // ac.sendGoal(goal);

  // bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(360.0));
  //
  // if (finishedBeforeTimeout)
  // {
  //   actionlib::SimpleClientGoalState state = ac.getState();
  //   ROS_INFO("Action finished: %s", state.toString().c_str());
  // }
  // else
  // {
  //   ROS_INFO("Action did not finish before time out.");
  // }

  /*
  backPose[0] = poseN;
  //poses[1] = pose1;
  goal.intermediatePoints = backPose;
  ac.sendGoal(goal);

  bool finishedBeforeTimeout2 = ac.waitForResult(ros::Duration(30.0));

  if (finishedBeforeTimeout2)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before time out.");
  }

  */
  return 0;

}
