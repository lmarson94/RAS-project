#include <ros/ros.h>
#include "AddToMap.h"
#include <deque>

AddToMap::AddToMap()
{

  obj_mark.header.frame_id = "base_map";
  obj_mark.header.stamp = ros::Time();
  obj_mark.type = visualization_msgs::Marker::CUBE;
  obj_mark.action = visualization_msgs::Marker::ADD;

  obj_mark.scale.x = 0.05;
  obj_mark.scale.y = 0.05;
  obj_mark.scale.z = 0.05;
  obj_mark.color.a = 1.0;
  obj_mark.color.r = 1.0;
  obj_mark.color.g = 0.0;
  obj_mark.color.b = 0.0;
  obj_mark.id = 0;

  text_mark.header.frame_id = "base_map";
  text_mark.header.stamp = ros::Time();
  text_mark.type = 9;
  text_mark.action = 0;

  text_mark.scale.x = 0.05;
  text_mark.scale.y = 0.05;
  text_mark.scale.z = 0.05;
  text_mark.color.a = 1.0;
  text_mark.color.r = 0.0;
  text_mark.color.g = 0.0;
  text_mark.color.b = 0.0;
  text_mark.id = 100;
}
std::string AddToMap::mapClassObject(int clasId) // maps the classification result to a corresponding String, describing the object
{
	//ras_msgs::RAS_Evidence msg;
	string map[15] = {ras_msgs::RAS_Evidence::yellow_ball, ras_msgs::RAS_Evidence::yellow_cube, ras_msgs::RAS_Evidence::green_cube, ras_msgs::RAS_Evidence::green_cylinder, ras_msgs::RAS_Evidence::green_hollow_cube, ras_msgs::RAS_Evidence::orange_cross, ras_msgs::RAS_Evidence::patric, ras_msgs::RAS_Evidence::red_cylinder, ras_msgs::RAS_Evidence::red_hollow_cube, ras_msgs::RAS_Evidence::red_ball, ras_msgs::RAS_Evidence::blue_cube, ras_msgs::RAS_Evidence::blue_triangle, ras_msgs::RAS_Evidence::purple_cross, ras_msgs::RAS_Evidence::purple_star, ras_msgs::RAS_Evidence::an_object};
	return map[clasId-1];
}
void AddToMap::objCallback(const ras_object_master::ObjectMasterMsg::ConstPtr& msg)
{

  obj_mark.pose.position.x = msg->point.point.x;
  obj_mark.pose.position.y = msg->point.point.y;
  obj_mark.pose.position.z = 0.0;
  obj_mark.id = msg->point.point.z;
  marker_pub.publish(obj_mark);
  //create textlabel
  text_mark.id = msg->point.point.z + 100;
  text_mark.pose.position.x = msg->point.point.x;
  text_mark.pose.position.y = msg->point.point.y;
  text_mark.pose.position.z = 0.0;
  stringstream ss;
  ss << mapClassObject(msg->clas) << ", " << msg->conf;
  text_mark.text = ss.str();
  marker_pub.publish(text_mark);
  //obj_mark.id++;
}


void AddToMap::main(int argc, char** argv)
{
  float xAvg;
  float yAvg;
  float zAvg;

  ros::init(argc, argv, "addMarker");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  obj_sub = n.subscribe("/target/Geometry/New_Point", 1, &AddToMap::objCallback, this);

  ros::spin();
}

int main(int argc, char **argv)
{
  AddToMap marker_obj = AddToMap();
  marker_obj.main(argc, argv);
}
