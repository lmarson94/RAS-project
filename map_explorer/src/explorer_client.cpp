#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <map_explorer/ExplorationAction.h>
#include "../../occupancy_grid/src/GridMap.h"

nav_msgs::OccupancyGrid exporationMap;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  exporationMap = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer_client");
  ros::NodeHandle n;
  ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);

  ROS_INFO("Waiting for occupancy grid");
  while (!exporationMap.data.size() > 0)
  {
    ros::spinOnce();
  }

  // Client part
  ROS_INFO("Starting client");
  actionlib::SimpleActionClient<map_explorer::ExplorationAction> ac("map_explorer", true);

  ROS_INFO("Waiting for server");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");

  double resolution = 0.03;      // Resolution [m]
  const int R1 = 5;                    // Inflation radius 1
  const int R2 = 7;                    // Inflation radius 2
  double width = 2.440000;
  double height = 2.425000;

  int n_width  = (int) ceil(width/resolution);
  int n_height = (int) ceil(height/resolution);

  /*GridMap grid = GridMap(n_width, n_height, resolution, "lab_maze_2018.txt");
  grid.inflateMap(R2, 25);
  grid.inflateMap(R1, 50);*/

  // send a goal to the action
  map_explorer::ExplorationGoal goal;
  goal.gridmap.map = exporationMap;
  goal.gridmap.n_width = n_width;
  goal.gridmap.n_height = n_height;
  goal.gridmap.resolution = resolution;
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}
