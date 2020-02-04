#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_detection/msg_objects_detected.h>
#include <ras_object_detection/PointStampedArray.h>
#include <tf/transform_listener.h>
#include <vector>
#include "ras_object_detection/EstimateObjectPosition.h"

tf::TransformListener* pListener = NULL;

class ObjectPositionEstimation
{
    public:
        ObjectPositionEstimation();
        void run (int argc, char **argv);
    private:
        //ros::Publisher pub_position;
        ros::ServiceServer service;
        struct boundingBox {
          int xul, yul, xdr, ydr;
        };
        int size;

        bool estimatePosition(ras_object_detection::EstimateObjectPosition::Request  &req,
                              ras_object_detection::EstimateObjectPosition::Response &res);
        geometry_msgs::PointStamped getAverageDepth(const sensor_msgs::PointCloud2 cloud, boundingBox b);
};
