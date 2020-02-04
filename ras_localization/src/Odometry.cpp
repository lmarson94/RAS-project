#include "Odometry.h"

//#include <ros/ros.h>
//#include <phidgets/motor_encoder.h>
//#include <geometry_msgs/Vector3.h>
//#include <math.h>

Odometry::Odometry()
{
  countLeftOld = 0;
  countRightOld = 0;
  countLeft = 0;
  countRight = 0;
}

void Odometry::leftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  //ROS_INFO("Left encoder heard: %d", msg->count);
  countLeft = msg->count;
}

void Odometry::rightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  //ROS_INFO("Right encoder heard: %d", msg->count);
  countRight = msg->count ;
}

void Odometry::run(int argc, char** argv) {
    int deLeft, deRight;

    float x, y, theta;

    // float x = 0.420653969049;
    // float y = 2.05006837845;
    // float theta = 4.18815469742;

    float dx, dy, dtheta;
    geometry_msgs::Vector3 msg;

    ros::init(argc, argv, "odometry");
    ros::NodeHandle n("~");

    n.getParam("/x", x);
    n.getParam("/y", y);
    n.getParam("/theta", theta);
    n.getParam("/params/updateFrequency", updateFrequency);
    n.getParam("/params/b", b);
    n.getParam("/params/r", r);
    n.getParam("/params/countsPerRevolution", countsPerRevolution);
    dt = 1.0/updateFrequency;

    ros::Rate loop_rate(updateFrequency);
    ros::Subscriber leftSub = n.subscribe("/left/encoder", 1, &Odometry::leftEncoderCallback, this);
    ros::Subscriber rightSub = n.subscribe("/right/encoder", 1, &Odometry::rightEncoderCallback, this);

    ros::Publisher pub_odom_increment = n.advertise<geometry_msgs::Vector3>("/localisation/odometry/increment", 1);
    ros::Publisher pub_odom = n.advertise<geometry_msgs::Vector3>("/localisation/odometry", 1);

    while(ros::ok()){

      ros::spinOnce();

      deLeft = countLeft - countLeftOld;
      // Right motor has been inverted to drive the same way as the left motor
      deRight = -(countRight - countRightOld);

      countLeftOld = countLeft;
      countRightOld = countRight;

      wL = deLeft * 2 * M_PI * updateFrequency / countsPerRevolution;
      wR = deRight * 2 * M_PI * updateFrequency / countsPerRevolution;

      v = (wL + wR) * r / 2.0;
      w = (wR - wL) * r / b;

      dx = v * dt * cos(theta);
      dy = v * dt * sin(theta);
      dtheta = w * dt;

      msg.x = dx;
      msg.y = dy;
      msg.z = dtheta;

      pub_odom_increment.publish(msg);

      x += dx;
      y += dy;
      theta += dtheta;

      msg.x = x;
      msg.y = y;
      msg.z = theta;

      pub_odom.publish(msg);

      loop_rate.sleep();
    }
}

int main(int argc, char** argv){

  Odometry odometry = Odometry();
  odometry.run(argc, argv);
  return 0;
}
