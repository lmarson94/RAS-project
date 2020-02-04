#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
//#include <actionlib/server/simple_done_callback.h> SHOULD ALREADY EXIST IN simple_action_client.h !!!
#include <map_explorer/ExplorationAction.h>
#include <pure_pursuit_controller/PurePursuitAction.h>
#include "../../occupancy_grid/src/OccupancyRegion.cpp"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"

/*
----------------------------- PROBLEM!!!!! --------------------------
THIS PART HAS TO REMEMBER WHAT HAS BEEN EXPLORED IF IT'S PREEMPTED!!!
********************************************************************
*/

int clamp(int value, int min, int max);

bool purePursuitDone = false;

void purePursuitDoneCallback(const actionlib::SimpleClientGoalState &state, const pure_pursuit_controller::PurePursuitResultConstPtr &result);

class ExplorationAction
{
  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<map_explorer::ExplorationAction> as_;
    std::string action_name_;
    map_explorer::ExplorationFeedback feedback_;
    map_explorer::ExplorationResult result_;
    ros::Subscriber localisationSub = nh_.subscribe<geometry_msgs::Vector3>("/localisation/position", 1, &ExplorationAction::localisationCallback, this);
    ros::Subscriber pathSub = nh_.subscribe<nav_msgs::Path>("/aPath", 1, &ExplorationAction::pathCallback, this);
    ros::Publisher pathPlanningPub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Publisher mapPub = nh_.advertise<nav_msgs::OccupancyGrid>("/explorationMap", 1);
    geometry_msgs::PoseStamped targetPose;
    nav_msgs::OccupancyGrid map;
    nav_msgs::Path path;
    geometry_msgs::Vector3 currentPos;
    int currentGridPos[2], n_width, n_height;
    double gridResolution = -1.0; // Negative value means that resolution hasn't  been initialised
    double lookAheadDistance = 0.5;
    bool hasReceivedPos = false;
    bool hasReceivedPath = false;

    void executeCB(const map_explorer::ExplorationGoalConstPtr &goal)
    {
      ROS_INFO("Received goal");
      // Sets initial parameters
      ros::Rate r(10);  // TODO: Get from parameter server?
      bool success = true;

      // TODO: USE XMAX AND YMAX INSTEAD, AND MAKE SURE THAT REGIONS DON'T CONTAIN POINTS OUTSIDE OF MAZE
      this->n_width = goal->gridmap.n_width;
      this->n_height = goal->gridmap.n_height;
      this->map = goal->gridmap.map;
      this->gridResolution = goal->gridmap.resolution;

      ROS_INFO("Waiting for robot position");
      while (!hasReceivedPos)
      {
        // TODO: MAKE SURE THAT IT'S NOT AN ENDLESS LOOP! FAIL AFTER SOME TIME?
        ros::spinOnce();
        r.sleep();
      }
      ROS_INFO("Robot position received");

      // Connects to pure pursuit server
      actionlib::SimpleActionClient<pure_pursuit_controller::PurePursuitAction> ac("purePursuit", true);
      ROS_INFO("Waiting for pure pursuit action server to start");
      ac.waitForServer();

      // Split occupancy grid into regions
      int regionSize = (n_width < n_height)? (int)(n_width/2) : (int)(n_height/2);
      int num_regions_x = (int)ceil(n_width/regionSize);
      int num_regions_y = (int)ceil(n_height/regionSize);
      int num_regions = num_regions_x * num_regions_y;
      std::vector<OccupancyRegion> regions;
      for (int i=0; i < n_height; i += regionSize)
      {
        for (int j=0; j < n_width; j += regionSize)
        {
          int actualWidth = std::min(regionSize, n_width-j);
          int actualHeight = std::min(regionSize, n_height-i);
          OccupancyRegion myRegion = OccupancyRegion(&map, actualWidth, actualHeight, n_width, j, i);
          regions.push_back(myRegion);
        }
      }
      // DEBUG: CURRENTLY USE WHOLE MAP AS REGIONS
      /*std::vector<OccupancyRegion> regions;
      int num_regions_x = 1;
      int num_regions_y = 1;
      int num_regions = num_regions_x * num_regions_y;
      OccupancyRegion myRegion = OccupancyRegion(&map, n_width, n_height, n_width, 0, 0);
      regions.push_back(myRegion);*/

      // Find index of region that is furthest away
      int regionIndex = 0;
      double maxDistanceToRegionsSqr = 0.0;
      std::vector<int> indicesToTry = {0, num_regions_x-1, (num_regions_y-1)*num_regions_x, num_regions_y*num_regions_x-1};  // Contains indices of all corner regions
      for (int i=0; i < indicesToTry.size(); i++)
      {
        double distanceSqr = pow(regions[i].midPoint[0] - currentGridPos[0], 2) + pow(regions[i].midPoint[1] - currentGridPos[0], 2);
        if (distanceSqr > maxDistanceToRegionsSqr)
        {
          regionIndex = indicesToTry[i];
          maxDistanceToRegionsSqr = distanceSqr;
        }
      }

      int furthestUnexploredIndex;
      int numExploredRegions = 0;

      ROS_INFO("Entering loop");
      while(numExploredRegions < num_regions)
      {
        furthestUnexploredIndex = regions[regionIndex].getFurthestUnexploredIndex(currentGridPos[0], currentGridPos[1]);
        while (furthestUnexploredIndex == -1 && numExploredRegions < num_regions)
        {
          ++numExploredRegions;
          regionIndex = (regionIndex == num_regions-1)? 0 : regionIndex+1;
          furthestUnexploredIndex = regions[regionIndex].getFurthestUnexploredIndex(currentGridPos[0], currentGridPos[1]);
        }
        if (numExploredRegions == num_regions) break;

        ROS_INFO("Next target found, waiting for path planner");
        // Find path to target pose
        targetPose.pose = cellToPose(furthestUnexploredIndex);
        ROS_INFO("Go to cell %d, pose: (%f, %f)", furthestUnexploredIndex, targetPose.pose.position.x, targetPose.pose.position.y);
        pathPlanningPub.publish(targetPose);
        this->hasReceivedPath = false;
        while (!hasReceivedPath)
        {
          ros::spinOnce();
          if (as_.isPreemptRequested() || !ros::ok())
          {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
            break;
          }
          r.sleep();
        }
        if (success == false)
        {
          break;
        }
        ROS_INFO("Path received, contacting pure pursuit server");

        // TODO: THINK THIS THROUGH! WOULDN'T IT BE BETTER IF BRAIN HANDLED THIS?
        // CAN BE SEND REFERENCES AS MESSAGES, TO NOT CONSTANTLY COPY GRIDMAPS?

        // Make pure pursuit go to goal and wait for it to finish
        pure_pursuit_controller::PurePursuitGoal purePursuitGoal;
        purePursuitGoal.intermediatePoints = this->path;
        // &ExplorationAction::localisationCallback, this
        purePursuitDone = false;
        ac.sendGoal(purePursuitGoal, purePursuitDoneCallback);

        ROS_INFO("Waiting for pure pursuit server");
        while (not purePursuitDone)
        {
          ros::spinOnce();
          if (as_.isPreemptRequested() || !ros::ok())
          {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            ac.cancelGoal();
            success = false;
            break;
          }
          r.sleep();
        }
        if (success == false)
        {
          break;
        }

        /*if (finishedBeforeTimeout)
        {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
        {
          ROS_INFO("Action did not finish before time out.");
        }*/

      }

      ROS_INFO("Finishing");
      this->gridResolution = -1.0;
      this->hasReceivedPos = false;
      this->hasReceivedPath = false;

      if(success)
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
      }

    }

    void localisationCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
      // TODO: MARK CELLS IN NEIGHBOURHOOD AS EXPLORED????
      // Waits until gridmap has been received and resolution initialised
      if( this->gridResolution > 0)
      {
        this->hasReceivedPos = true;
        this->currentPos = *msg;
        this->currentGridPos[0] = (int)(currentPos.x/this->gridResolution);
        this->currentGridPos[1] = (int)(currentPos.y/this->gridResolution);
        double cosTheta = cos(currentPos.z);
        double sinTheta = sin(currentPos.z);
        int dy = (int)(this->lookAheadDistance*sinTheta/this->gridResolution);
        int dx = (int)(this->lookAheadDistance*cosTheta/this->gridResolution);
        int cellIndex = this->currentGridPos[1]*this->n_width + this->currentGridPos[0];
        int cellVisionIndex = (currentGridPos[1]+dy)*n_width + currentGridPos[0] + dx;
        int visionSquareWidth = max(dx, dy);
        // TODO: Remove hard-coding of rectangle size
        markCellsAsExplored(cellIndex, 11, 11);
        // Marks a rectable representing vision of robot as explored
        // ROS_INFO("num cells: %d", n_width*n_height);
        // ROS_INFO("Vision index: %d, width: %d", cellVisionIndex, visionSquareWidth);
        markCellsAsExplored(cellVisionIndex, visionSquareWidth, visionSquareWidth);
        mapPub.publish(this->map);
        // DEBUG
        feedback_.feedback = map;
        as_.publishFeedback(feedback_);
      }
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
      ROS_INFO("Received a path! Not doing anything with it though");
      this->hasReceivedPath = true;
      this->path = *msg;
      // TODO: IS THIS ENOUGH?
    }

    geometry_msgs::Pose cellToPose(int index)
    {
      // TODO: Check that this is actually correct
      int xIndex = (int)(index%this->n_width);
      int yIndex = (int)((index-xIndex)/this->n_width);
      double xCoord = xIndex*this->gridResolution + 0.5*this->gridResolution;
      double yCoord = yIndex*this->gridResolution + 0.5*this->gridResolution;
      geometry_msgs::Pose pose;
      pose.position.x = xCoord;
      pose.position.y = yCoord;
      return pose;
    }

    // Marks cells as explored in a rectable around centerCellIndex
    void markCellsAsExplored(int centerCellIndex, int width, int height)
    {
      int x_index = centerCellIndex%this->n_width;
      int y_index = (int)(centerCellIndex/n_width);
      int x_start = x_index - (int)floor(width/2);
      int y_start = y_index - (int)floor(height/2);
      // i and j might be outside of the boundaries of the map
      for (int i = y_start; i < y_start + height; i++)
      {
        for (int j = x_start; j < x_start + width; j++)
        {
          int row = clamp(i, 0, n_height-1);
          int col = clamp(j, 0, n_width -1);
          // TODO: Remove hard-coded 50?
          if (this->map.data[row*n_width+col] < 50 && this->map.data[row*n_width+col] > -2)
          {
            map.data[row*n_width+col] = -2;
          }
        }
      }
    }

  public:

    ExplorationAction(std::string name) :
      as_(nh_, name, boost::bind(&ExplorationAction::executeCB, this, _1), false),
      action_name_(name)
    {
      as_.start();
    }

};

void purePursuitDoneCallback(const actionlib::SimpleClientGoalState &state, const pure_pursuit_controller::PurePursuitResultConstPtr &result)
{
  purePursuitDone = true;
}

int clamp(int value, int min, int max)
{
  if (value > max) value = max;
  if (value < min) value = min;
  return value;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer_server");
  ExplorationAction map_explorer("map_explorer");
  ros::spin();

  return 0;
}
