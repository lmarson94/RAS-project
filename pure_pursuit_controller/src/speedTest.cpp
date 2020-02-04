#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

class SpeedTester
{
private:
  ros::Publisher leftPub;
  ros::Publisher rightPub;
  ros::Subscriber velSub;
  int updateFrequency;
  float desiredVel;

  void Callback(const std_msgs::Float32::ConstPtr& msg);

public:
  SpeedTester(int argc, char** argv);
  void Run();
};

SpeedTester::SpeedTester(int argc, char** argv)
{
    ros::init(argc, argv, "speedTester");
    updateFrequency = 10;
    desiredVel = 0.0;

}

void SpeedTester::Callback(const std_msgs::Float32::ConstPtr& msg)
{
  desiredVel = msg->data;
}

void SpeedTester::Run()
{
  ros::NodeHandle n;
  leftPub = n.advertise<std_msgs::Float32>("/left/cmd_vel", updateFrequency);
  rightPub = n.advertise<std_msgs::Float32>("/right/cmd_vel", updateFrequency);
  velSub = n.subscribe("/speed_tester/vel", 1, &SpeedTester::Callback, this);
  ros::Rate loop_rate(updateFrequency);
  std_msgs::Float32 msgLeft;
  std_msgs::Float32 msgRight;

  while(ros::ok())
  {
    ros::spinOnce();
    msgLeft.data = desiredVel;
    msgRight.data = desiredVel;
    leftPub.publish(msgLeft);
    rightPub.publish(msgRight);
    ROS_INFO("Publishing: %f", desiredVel);

    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  SpeedTester tester = SpeedTester(argc, argv);
  tester.Run();
}
