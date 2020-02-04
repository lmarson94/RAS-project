#include <ros/ros.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <std_msgs/String.h>

class Odometry
{
    public:
      Odometry();
      void run(int argc, char** argv);

    private:
      double v, w, wL, wR, b, r, countsPerRevolution, dt;
      int countLeftOld;
      int countRightOld;
      int countLeft;
      int countRight;
      int updateFrequency;
      float locX, locY, locTheta;
      std_msgs::String msg;

      void odomCallback(const geometry_msgs::Vector3::ConstPtr& msg);
      void leftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg);
      void rightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg);
      void localisationCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};
