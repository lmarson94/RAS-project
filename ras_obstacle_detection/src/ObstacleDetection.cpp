#include "ObstacleDetection.h"

ObstacleDetection::ObstacleDetection () {
}

void ObstacleDetection::pointCloudCallBack (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::PassThrough<pcl::PCLPointCloud2> filter;
  filter.setInputCloud (cloudPtr);
  filter.setFilterFieldName ("y");
  filter.setFilterLimits (-100.0, 0.10);
  filter.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

void ObstacleDetection::run (int argc, char **argv) {
  ros::init (argc, argv, "object_position_estimation");
  ros::NodeHandle n;

  pub = n.advertise<sensor_msgs::PointCloud2> ("/camera/depth/filtered_points", 1);
  ros::Subscriber sub = n.subscribe ("/camera/depth_registered/points", 1, &ObstacleDetection::pointCloudCallBack, this);

  ros::spin ();
}

/*----------------------------------------------------------------------------*/

int main (int argc, char **argv) {
  ObstacleDetection detector = ObstacleDetection();
  detector.run(argc, argv);
}
