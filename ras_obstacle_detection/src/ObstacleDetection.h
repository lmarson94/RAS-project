#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Point32.h>

class ObstacleDetection
{
    public:
        ObstacleDetection ();
        void run (int argc, char **argv);
    private:
        ros::Publisher pub;

        void pointCloudCallBack (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

};
