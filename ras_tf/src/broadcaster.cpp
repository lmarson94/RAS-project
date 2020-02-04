#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

float x, y, theta;
float xo, yo, thetao;

void odometryCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  x = msg->x;
  y = msg->y;
  theta = msg->z;
}

//void oCallback(const geometry_msgs::Vector3::ConstPtr& msg)
//{
//  xo = msg->x;
//  yo = msg->y;
//  thetao = msg->z;
//}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  ros::Subscriber sub_left_encoder = n.subscribe("/localisation/position", 1, odometryCallback);
  //ros::Subscriber sub_odom = n.subscribe("/localisation/odometry", 1, oCallback);

  tf::TransformBroadcaster br;
  tf::Transform map_to_link_transform, map_to_odom;
  tf::Quaternion q, q1;;

  while(n.ok()){
    ros::spinOnce();

    map_to_link_transform.setOrigin( tf::Vector3(x, y, 0.0) );
    q.setRPY(0.0, 0.0, theta);
    map_to_link_transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(map_to_link_transform, ros::Time::now(), "base_map", "base_link"));

    //map_to_odom.setOrigin( tf::Vector3(xo, yo, 0.0) );
    //q1.setRPY(0.0, 0.0, thetao);
    //map_to_odom.setRotation(q1);
    //br.sendTransform(tf::StampedTransform(map_to_odom, ros::Time::now(), "base_map", "odom"));

    loop_rate.sleep();
  }

}
