#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ras_object_detection/PointStampedArray.h>
#include <tf/tf.h>
#include <ras_object_master/ObjectMasterMsg.h>
#include <ras_msgs/RAS_Evidence.h>

#include <cmath>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

class AddToMap
{

  public:
    void main(int argc, char **argv);
    AddToMap();

  private:

    visualization_msgs::Marker obj_mark, text_mark;

    ros::Publisher marker_pub;
    ros::Subscriber obj_sub;

    void objCallback(const ras_object_master::ObjectMasterMsg::ConstPtr& msg);
    string mapClassObject(int clasId);
};
