#include "ros/ros.h"
#include "impossible_object_pickup/PickupImpossibleObject.h"
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "pickup_impossible_object_client");
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<impossible_object_pickup::PickupImpossibleObject>("pickup_impossible_object");
   impossible_object_pickup::PickupImpossibleObject srv;
   srv.request.object_pos.x = 0.4;
   srv.request.object_pos.y = 0.88;
   if (client.call(srv))
   {
      ROS_INFO("Response vector 1: (%f, %f)", srv.response.point1.x, srv.response.point1.y);
      ROS_INFO("Response vector 2: (%f, %f)", srv.response.point2.x, srv.response.point2.y);
   }
   else
   {
      ROS_INFO("Failed!");
      //return 1;
   }
   double r = 0.015;
   visualization_msgs::Marker point0Marker;
   visualization_msgs::Marker point1Marker;
   visualization_msgs::Marker point2Marker;
   ros::Publisher point1pub = n.advertise<visualization_msgs::Marker>("/point1marker", 1);
   ros::Publisher point2pub = n.advertise<visualization_msgs::Marker>("/point2marker", 1);
   ros::Publisher point0pub = n.advertise<visualization_msgs::Marker>("/point0marker", 1);

   // Setting up markers
   point1Marker.header.frame_id = "/base_map"; point2Marker.header.frame_id = "/base_map"; point0Marker.header.frame_id = "/base_map";
   point1Marker.header.stamp = ros::Time::now(); point2Marker.header.stamp = ros::Time::now(); point0Marker.header.stamp = ros::Time::now();
   point1Marker.ns = "point1maker"; point2Marker.ns = "point2maker"; point0Marker.ns = "point2maker";
   point1Marker.id = 0; point2Marker.id = 0; point0Marker.id = 0;

   point1Marker.type = visualization_msgs::Marker::SPHERE;
   point1Marker.action = visualization_msgs::Marker::ADD;
   point1Marker.pose.position.x = srv.response.point1.x; point1Marker.pose.position.y = srv.response.point1.y; point1Marker.pose.position.z = 0;
   point1Marker.pose.orientation.x = 0.0; point1Marker.pose.orientation.y = 0.0; point1Marker.pose.orientation.z = 0.0; point1Marker.pose.orientation.w = 1.0;
   point1Marker.scale.x = 2*r; point1Marker.scale.y = 2*r; point1Marker.scale.z = 2*r;
   point1Marker.color.r = 0.0f; point1Marker.color.g = 0.0f; point1Marker.color.b = 1.0f; point1Marker.color.a = 0.5;
   point1Marker.lifetime = ros::Duration();

   point2Marker.type = visualization_msgs::Marker::SPHERE;
   point2Marker.action = visualization_msgs::Marker::ADD;
   point2Marker.pose.position.x = srv.response.point2.x; point2Marker.pose.position.y = srv.response.point2.y; point2Marker.pose.position.z = 0;
   point2Marker.pose.orientation.x = 0.0; point2Marker.pose.orientation.y = 0.0; point2Marker.pose.orientation.z = 0.0; point2Marker.pose.orientation.w = 1.0;
   point2Marker.scale.x = 2*r; point2Marker.scale.y = 2*r; point2Marker.scale.z = 2*r;
   point2Marker.color.r = 0.0f; point2Marker.color.g = 1.0f; point2Marker.color.b = 0.0f; point2Marker.color.a = 0.5;
   point2Marker.lifetime = ros::Duration();

   point0Marker.type = visualization_msgs::Marker::SPHERE;
   point0Marker.action = visualization_msgs::Marker::ADD;
   point0Marker.pose.position.x = srv.request.object_pos.x; point0Marker.pose.position.y = srv.request.object_pos.y; point0Marker.pose.position.z = 0;
   point0Marker.pose.orientation.x = 0.0; point0Marker.pose.orientation.y = 0.0; point0Marker.pose.orientation.z = 0.0; point0Marker.pose.orientation.w = 1.0;
   point0Marker.scale.x = 2*r; point0Marker.scale.y = 2*r; point0Marker.scale.z = 2*r;
   point0Marker.color.r = 1.0f; point0Marker.color.g = 0.0f; point0Marker.color.b = 0.0f; point0Marker.color.a = 0.5;
   point0Marker.lifetime = ros::Duration();

   while (true) {
     ros::spinOnce();
     point1pub.publish(point1Marker);
     point2pub.publish(point2Marker);
     point0pub.publish(point0Marker);
     ros::Duration(0.1).sleep();
   }

   return 0;
}
