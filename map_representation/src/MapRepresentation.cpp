#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include "MapRepresentation.h"
#include <cmath>

#include <iostream>
#include <fstream>

using namespace std;


MapRepresentation::MapRepresentation()
{
  wall_id = 0;

  wall.header.frame_id = "base_map";
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

  x_max = y_max = 0.0;
  x_least = y_least = 9999.0;
}

void MapRepresentation::updateMinMaxValues(double x1, double y1, double x2, double y2)
{
  // Min values
  if (x1 < x_least) x_least = x1;

  if (y1 < y_least) y_least = y1;

  if (x2 < x_least) x_least = x2;

  if (y2 < y_least) y_least = y2;

  // Max
  if (x1 > x_max) x_max = x1;
  if (x2 > x_max) x_max = x2;
  if (y1 > y_max) y_max = y1;
  if (y2 > y_max) y_max = y2;
}

double MapRepresentation::calc_dist(double x1, double x2, double y1, double y2)
{
  double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
  return dist;
}

double MapRepresentation::calc_angle(double x1, double x2, double y1, double y2)
{
    double angle = atan2(y2-y1,x2-x1);
    return angle;
}

void MapRepresentation::addWalls(std::vector<i4tuple> Walls, std::vector<string> text)
{
  int idx = 0;
  for(i4tuple t: Walls)
  {
    double x1 = std::get<0>(t);
    double y1 = std::get<1>(t);
    double x2 = std::get<2>(t);
    double y2 = std::get<3>(t);

    x1 -= x_least;
    x2 -= x_least;
    y1 -= y_least;
    y2 -= y_least;

    double dist = calc_dist(x1,x2,y1,y2);
    double angle = calc_angle(x1,x2,y1,y2);

    wall.scale.x = std::max(0.01,dist);
    wall.pose.position.x = (x1+x2)/2;
    wall.pose.position.y = (y1+y2)/2;
    wall.text=text[idx];
    tf::Quaternion q; q.setRPY(0.0,0.0,angle); // (roll(around x), pitch(around y), yaw(around z))
    tf::quaternionTFToMsg(q, wall.pose.orientation);
    wall.id = wall_id;
    walls.markers.push_back(wall);

    wall_id++;
    idx++;
  }
}

void MapRepresentation::main(int argc, char** argv)
{
  ros::init(argc, argv, "MapRepresentation");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);
  grid_size_pub = n.advertise<geometry_msgs::Point>("/grid_size",0);

  n.param<string>("map_file", _map_file, "maze_map.txt");

  ifstream map_fs; map_fs.open(_map_file.c_str());

  std::vector<MapRepresentation::i4tuple> walls_;
  std::vector<string> text_vec;
  while (getline(map_fs, line)){


      if (line[0] == '#') {
          continue;
      }

    std::istringstream line_stream(line);
    line_stream >> x1 >> y1 >> x2 >> y2;
    updateMinMaxValues(x1, y1, x2, y2);
    text_vec.push_back(line_stream.str());
    walls_.push_back(MapRepresentation::i4tuple(x1,y1,x2,y2));
  }
  addWalls(walls_,text_vec);

  if (x_least < 0){
    p.x = x_max - x_least;
  }
  else{
    p.x = x_max + x_least;
  }
  if (y_least < 0){
    p.y = y_max - y_least;
  }
  else{
    p.y = y_max + y_least;
  }

    ros::Rate r(10);
    while (ros::ok())
    {

      marker_pub.publish(walls);
      grid_size_pub.publish(p);
      ros::spinOnce();
      r.sleep();

    }
    return;
}

int main(int argc, char **argv)
{
  MapRepresentation map_repres = MapRepresentation();
  map_repres.main(argc, argv);
}
