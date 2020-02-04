#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <vector>
#include <math.h>

//tf::TransformListener* pListener = NULL;


class BatteryDetection
{
	public:
		BatteryDetection();
		void detect (int argc, char **argv);
	private:

		struct a_v {
			int xul, yul, xlr, ylr;
		};
		int min_width_v, max_width_v;
		
		struct a_h {
			int xul, yul, xlr, ylr;
		};
		int min_width_h, max_width_h;
		
		a_v area_v;
		a_h area_h;

		ros::Subscriber pclSub;
		ros::Publisher posPub; 
		
		void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

};	
