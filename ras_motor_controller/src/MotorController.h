#include "ros/ros.h"
#include "phidgets/motor_encoder.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <tuple>
#include <math.h>
#include <sstream>

class MotorController
{
  public:
    void run(int argc, char **argv);
    MotorController();
  private:
    int updateFrequency, EncoderR, EncoderL, dEncoderR, dEncoderL;
    double Kp_l, Ki_l, Kd_l, Kp_r, Ki_r, Kd_r, rInv, b, dt, v, w, intErrorR, intErrorL, countsPerRevolution, radPerSecPerCount;
    double error_last_l, error_last_r, error_now_l, error_now_r;
    double leftPWM, rightPWM;
    double t_last, t_now;
    double wMax, intCap;
    int count_last_r, count_last_l, count_now_l, count_now_r, counter, ctd;
    int arrR[10] = {};
    int arrL[10] = {};
    ros::Publisher leftPub;
    ros::Publisher rightPub;
    ros::Subscriber twistSub;
    ros::Subscriber leftEncoderSub;
    ros::Subscriber rightEncoderSub;
    int stopCounter;

    std::tuple<std_msgs::Float32, std_msgs::Float32> getControllerOutputs();
    std::tuple<std_msgs::Float32, std_msgs::Float32> getEncoderTestOutputs();
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void leftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg);
    void rightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg);
};

template <typename T> int sgn(T val);
