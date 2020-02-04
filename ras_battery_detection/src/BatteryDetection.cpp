#include "BatteryDetection.h"

BatteryDetection::BatteryDetection() {
	//Area for vertical position 
	area_v.xul = 0;
	area_v.yul = 440;
	area_v.xlr = 640;
	area_v.ylr = 441; //maybe more and average
	min_width_v = 60;
	max_width_v = 180;
	//Area for horizontal position 
	area_h.xul = 30;
	area_h.yul = 180;
	area_h.xlr = 610;
	area_h.ylr = 181;
	min_width_h = 180;
	max_width_h = 400;
}

void BatteryDetection::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	ROS_INFO("####################");
	ROS_INFO("####################");
	ROS_INFO("####################");
	float x, y, z;
	float dist;
	int xOff = msg->fields[0].offset;
  	int yOff = msg->fields[1].offset;
  	int zOff = msg->fields[2].offset;

	int j = 500;
	for (int i = area_v.xul; i < area_v.xlr; i++) {
		//for (int j = area_v.yul; j < area_v.ylr; j++) {
        	memcpy(&x, &(msg->data[((msg->width) * j + i) * msg->point_step + xOff]), sizeof(float));
		memcpy(&y, &(msg->data[((msg->width) * j + i) * msg->point_step + yOff]), sizeof(float));
		memcpy(&z, &(msg->data[((msg->width) * j + i) * msg->point_step + zOff]), sizeof(float));
		if(!isnan(x) && !isnan(y) && !isnan(z)) {		
			dist = sqrt(x*x+y*y+z*z);
			ROS_INFO("No. %d: %f %f %f",i, x, y, z);
			ROS_INFO("No. %d: %f", i, dist);
		}		
		//}
	}
	ROS_INFO("Finished");
	return;
}

void BatteryDetection::detect(int argc, char **argv) {
	ros::init (argc, argv, "battery_detection");
  	ros::NodeHandle n;
	
  	//pListener = new (tf::TransformListener);
	
  	posPub = n.advertise<geometry_msgs::PointStamped> ("/position_battery", 1);
	pclSub = n.subscribe("/camera/depth_registered/points", 1, &BatteryDetection::pclCallback, this);

  	ros::Rate lr(10);

  	while(ros::ok())
  	{
      		ros::spinOnce();
      		lr.sleep();
  	}
}


int main (int argc, char **argv) {
  	BatteryDetection detector = BatteryDetection();
  	detector.detect(argc, argv);
}
