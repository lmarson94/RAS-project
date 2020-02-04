#include "LidarObstacleDetection.h"

LidarObstacleDetecion::LidarObstacleDetecion () {
  staticPub.data = "Object detected";
}

void LidarObstacleDetecion::laserCallback (const sensor_msgs::LaserScanConstPtr& laser_msg) {
  int points = 0;

  for(int i=157; i<202; i++) {
    if(!isinf(laser_msg->ranges[i]) && laser_msg->ranges[i] < 0.22) {
      points++;
    }
  }
  ROS_INFO("p %d", points);
  if(points > 5) {
      ROS_INFO("Obstacle detected in front of robot!");
      obj_det_pub.publish(staticPub);
  }

}

void LidarObstacleDetecion::run (int argc, char **argv) {
  ros::init (argc, argv, "lidar_obstacle_detection");
  ros::NodeHandle n("~");
  n.getParam("/params/updateFrequency", updateFrequency);
  obj_det_pub = n.advertise<std_msgs::String>("/obstacle_detected", updateFrequency);
  ros::Subscriber sub = n.subscribe ("/scan", 1, &LidarObstacleDetecion::laserCallback, this);
  ros::spin ();
}

/*----------------------------------------------------------------------------*/

int main (int argc, char **argv) {
  LidarObstacleDetecion detector = LidarObstacleDetecion();
  detector.run(argc, argv);
}
