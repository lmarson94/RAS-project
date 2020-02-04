#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

#include <cmath>

#include <iostream>
#include <fstream>

using namespace std;

double calc_dist(double x1, double x2, double y1, double y2)
{
  double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
  return dist;
}

double calc_angle(double x1, double x2, double y1, double y2)
{
    double angle = atan2(y2-y1,x2-x1);
    return angle;
}

// void mark_callback(visualization_msgs::MarkerArray Marker)
// {
//   //int _frame_id = msg->header.frame_id;
//   ROS_INFO("HEARED frame_id: %s", Marker);
// }

int main( int argc, char** argv )
{
  ros::init(argc, argv, "map_representation");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);
  //ros::Subscriber marker_sub = n.subscribe("visualization_marker_array", 10, mark_callback);

  string _map_file;
  n.param<string>("map_file", _map_file, "maze_map.txt");

  ifstream map_fs; map_fs.open(_map_file.c_str());

  visualization_msgs::Marker wall;
  visualization_msgs::MarkerArray walls;


  wall.header.frame_id = "/maze_map_representation";
  wall.header.stamp = ros::Time();
  wall.ns = "map_representation";
  wall.type = visualization_msgs::Marker::CUBE;
  wall.action = visualization_msgs::Marker::ADD;

  wall.pose.position.z = 0.1;
  wall.scale.y = 0.01;
  wall.scale.z = 0.2;
  wall.color.a = 1.0;
  wall.color.r = 0.0;
  wall.color.g = 1.0;
  wall.color.b = 0.0;

  int count = 0;
  int wall_id = 0;

  string line;
  double x1,x2,y1,y2;
  while (getline(map_fs, line)){

      if (line[0] == '#') {
          // comment -> skip
          continue;
      }

    std::istringstream line_stream(line);
    line_stream >> x1 >> y1 >> x2 >> y2;

    ROS_INFO("(x1,y1) = (%f,%f) (x2,y2) = (%f,%f)", x1,y1,x2,y2);

    double dist = calc_dist(x1,x2,y1,y2);
    double angle = calc_angle(x1,x2,y1,y2);

    ROS_INFO("dist= %f, angle=%f", dist, angle);

    wall.scale.x = std::max(0.01,dist);
    wall.pose.position.x = (x1+x2)/2;
    wall.pose.position.y = (y1+y2)/2;
    wall.text=line_stream.str();
    tf::Quaternion q; q.setRPY(0.0,0.0,angle); // (roll(around x), pitch(around y), yaw(around z))
    tf::quaternionTFToMsg(q, wall.pose.orientation);

    wall.id = wall_id;
    wall_id++;
    walls.markers.push_back(wall);

    count++;
  }

  ros::Rate r(10);
  while (ros::ok())
  {
    //ROS_INFO("PUBLISHING WALLS");
    marker_pub.publish(walls);
    ros::spinOnce();
    r.sleep();

  }
}
