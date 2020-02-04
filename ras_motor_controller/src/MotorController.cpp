//#include "ros/ros.h"
//#include "phidgets/motor_encoder.h"
#include "MotorController.h"
//#include "geometry_msgs/Twist.h"
//#include "std_msgs/Float32.h"
//#include <message_forward.h>
//#include <tuple>
//#include <math.h>
//#include <sstream>

MotorController::MotorController()
{

    updateFrequency = 10;
    count_last_l = 0;
    count_last_r = 0;
    count_now_l = 0;
    count_now_r = 0;


    stopCounter = 0;
    /*Kp_l = 2.0;
    Ki_l = 17.0;
    Kd_l = 0.0;
    Kp_r = 2.32;
    Ki_r = 17.0;
    Kd_r = 0.0;*/
    /*
    rInv = 1.0/(0.098*0.5);
    b = 0.22;
    countsPerRevolution = 1185.458; //3591.84
    radPerSecPerCount = updateFrequency / countsPerRevolution;
    dt = 1.0/updateFrequency;*/
    t_last = 0;
    t_now = 0;
    v = 0.0;
    w = 0.0;
    intErrorR = 0;
    leftPWM = 0;
    rightPWM = 0;
    intErrorL = 0;
    EncoderR = 0;
    EncoderL = 0;
    dEncoderR = 0;
    dEncoderL = 0;
    counter = 0;
    error_last_l = 0;
    error_last_r = 0;
    error_now_l = 0;
    error_now_r = 0;
    counter = 0;
    ctd = 0;
}

void MotorController::leftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    stopCounter = 0;
    EncoderL = msg->count;
    ROS_INFO("Left encoder heard:  %d", EncoderL);
}

void MotorController::rightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    stopCounter = 0;
    EncoderR = msg->count;
    ROS_INFO("Right encoder heard: %d", EncoderR);
}

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    v = msg->linear.x;
    w = msg->angular.z;
    //ROS_INFO("Twist heard: %f, %f", v, w);
}

std::tuple<std_msgs::Float32, std_msgs::Float32> MotorController::getEncoderTestOutputs()
{
	//To be continued

	std_msgs::Float32 msgLeft;
	std_msgs::Float32 msgRight;

	leftPWM = 20;
	rightPWM = 0;

	if (EncoderL > (ctd-135))
	{
		leftPWM = 0;
		rightPWM = 0;
		ROS_INFO("Program stopped");
	}

	msgLeft.data = leftPWM;
	msgRight.data = rightPWM;

	std::tuple<std_msgs::Float32, std_msgs::Float32> msg_pair = std::make_tuple(msgLeft, msgRight);
  return msg_pair;
}

std::tuple<std_msgs::Float32, std_msgs::Float32> MotorController::getControllerOutputs()
{
    /*std_msgs::Float32 msgLeft;
    std_msgs::Float32 msgRight;
    msgLeft.data = 50;
    msgRight.data = 50;
    std::tuple<std_msgs::Float32, std_msgs::Float32> msg_pair = std::make_tuple(msgLeft, msgRight);
    return msg_pair;*/

    count_last_l = count_now_l;
    count_now_l = EncoderL;
    count_last_r = count_now_r;
    count_now_r = EncoderR;
    dEncoderL = (double)(count_now_l - count_last_l);
    // Right motor is launched inverted, so that the motors spin in the same direction for the same input
    dEncoderR = -(double)(count_now_r - count_last_r);
    std_msgs::Float32 msgLeft;
    std_msgs::Float32 msgRight;
    t_last = t_now;
    t_now = ros::Time::now().toSec();
    dt = t_now - t_last;
    ROS_INFO("Delta t:  %f", dt);

    double desired_wl = rInv*(v - b*w / 2.0);
    double desired_wr = rInv*(v + b*w / 2.0);
    // Makes sure desired wheel angular velocities are withing achievable range
    if (fabs(desired_wl) > wMax) { desired_wl = sgn(desired_wl) * wMax; }
    if (fabs(desired_wr) > wMax) { desired_wr = sgn(desired_wr) * wMax; }
    // 1/18 = 0.05555555556
    double estimated_wl = 2.0 * M_PI * dEncoderL / (countsPerRevolution * dt);
    double estimated_wr = 2.0 * M_PI * dEncoderR / (countsPerRevolution * dt);

    intErrorL = intErrorL + dt * (desired_wl - estimated_wl);
    intErrorR = intErrorR + dt * (desired_wr - estimated_wr);
    if (fabs(intErrorR) > intCap)
    {
      intErrorR = intCap*sgn(intErrorR);
    }
    if (fabs(intErrorL) > intCap)
    {
      intErrorL = intCap*sgn(intErrorL);
    }
    error_last_l = error_now_l;
    error_now_l = desired_wl - estimated_wl;
    error_last_r = error_now_r;
    error_now_r = desired_wr - estimated_wr;

    //msgLeft.data = desired_wl + Ki * intErrorL;
    //msgRight.data = desired_wr + Ki * intErrorR;

    ROS_INFO("Kp L R:  %f %f", Kp_l, Kp_r);
    ROS_INFO("intErrorL, intErrorR: %f, %f", intErrorL, intErrorR);

    if(desired_wl != 0.0) {
      leftPWM = Kp_l*(desired_wl - estimated_wl) + Ki_l * intErrorL + Kd_l * (error_now_l - error_last_l);
    } else {
      intErrorL = 0.0;
      leftPWM = 0.0;
    }
    if(desired_wr != 0.0) {
      rightPWM = Kp_r*(desired_wr - estimated_wr) + Ki_r * intErrorR + Kd_r * (error_now_r - error_last_r);
    } else {
      intErrorR = 0.0;
      rightPWM = 0.0;
    }


    /*if (leftPWM > 100.0) {
      leftPWM = 100.0;
    }
    if (rightPWM > 100.0) {
      rightPWM = 100.0;ros::NodeHandle
    }*/

	  msgLeft.data = leftPWM;
	  msgRight.data = rightPWM;
	  ROS_INFO("Desired L R:  %f %f", desired_wl, desired_wr);
	  ROS_INFO("Estimated L R:  %f %f", estimated_wl, estimated_wr);
  	ROS_INFO("PWM L R:  %f %f", msgLeft.data, msgRight.data);
	  ROS_INFO("Encoder buffer L R: %d %d", dEncoderL, dEncoderR);
    ROS_INFO("-----------------------------------------");
    //dEncoderL = 0;
    //dEncoderR = 0;
    ++stopCounter;
    if (stopCounter >= 2)
    {
      v = 0;
      w = 0;
      EncoderL = 0;
      EncoderR = 0;
    }


    std::tuple<std_msgs::Float32, std_msgs::Float32> msg_pair = std::make_tuple(msgLeft, msgRight);
    return msg_pair;
}

void MotorController::run(int argc, char **argv)
{
    std_msgs::Float32 leftMotorMsg, rightMotorMsg;
    ros::init(argc, argv, "MotorController");

    ros::NodeHandle n("~");
    double r;
    n.getParam("gains_left_motor/Kp", Kp_l);
    ROS_INFO("%f", Kp_l);
    n.getParam("gains_left_motor/Ki", Ki_l);
    n.getParam("gains_left_motor/Kd", Kd_l);
    n.getParam("gains_right_motor/Kp", Kp_r);
    n.getParam("gains_right_motor/Ki", Ki_r);
    n.getParam("gains_right_motor/Kd", Kd_r);
    n.getParam("ctd", ctd);
    n.getParam("w_max", wMax);
    n.getParam("/params/updateFrequency", updateFrequency);
    n.getParam("/params/b", b);
    n.getParam("/params/r", r);
    n.getParam("/params/countsPerRevolution", countsPerRevolution);
    rInv = 1.0/r;
    // Maximum magnitude allowed for integral term
    intCap = 100.0/std::min(Ki_l, Ki_r);
    //dt = 1.0/updateFrequency;
    //radPerSecPerCount = updateFrequency / countsPerRevolution;

    leftPub = n.advertise<std_msgs::Float32>("/left/cmd_vel", updateFrequency);
    rightPub = n.advertise<std_msgs::Float32>("/right/cmd_vel", updateFrequency);
    twistSub = n.subscribe("/motor_controller/twist", 1, &MotorController::twistCallback, this);
    leftEncoderSub = n.subscribe("/left/encoder", 1, &MotorController::leftEncoderCallback, this);
    rightEncoderSub = n.subscribe("/right/encoder", 1, &MotorController::rightEncoderCallback, this);

    ros::Rate loop_rate(updateFrequency);
    //  boost::shared_ptr<phidgets::motor_encoder> m = ros::topic::waitForMessage("/left/encoder", 1000);
    //    ROS_INFO("Received Encoder Start Value:  %d", m.count);
    //  return;
    while(ros::ok())
    {
        ros::spinOnce();
        //std::tie(leftMotorMsg, rightMotorMsg) = getControllerOutputs();
        std::tie(leftMotorMsg, rightMotorMsg) = getControllerOutputs();
        leftPub.publish(leftMotorMsg);
        rightPub.publish(rightMotorMsg);

        loop_rate.sleep();

    }
    return;
}

// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char **argv)
{
    MotorController motorController = MotorController();
    motorController.run(argc, argv);
}
