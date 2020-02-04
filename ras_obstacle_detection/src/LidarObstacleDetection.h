#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

class LidarObstacleDetecion
{
    public:
        LidarObstacleDetecion ();
        void run (int argc, char **argv);
    private:
        ros::Publisher obj_det_pub;
        int updateFrequency;
        std_msgs::String staticPub;
        void laserCallback (const sensor_msgs::LaserScanConstPtr& laser_msg);
};
