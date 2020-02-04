#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <pure_pursuit_controller/PurePursuitAction.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h" // DEBUG: Calibration
#include <visualization_msgs/Marker.h>

// Ugly solution, but it works. I couldn't figure out how to make CMake compile
// a .cpp file that wasn't an executable, so I just included it in here
#include "LineSegment.cpp"

enum class State {DRIVE, TURN, ORIENTING, FINISHED};

class PurePursuitAction
{
  private:
        // TODO: Implement D-term, but make sure to use low-pass filter
        State state;
        double r, rInner, stopDistance, Kp, Ki, Kd, smoothAngularErrorOld;
        double desiredV, maxDesV, desiredHeadingOld, switchToDriveThreshold, dt;
        double unconstrainedErrorAddition, switchToTurnThreshold, closeToGoalDist;
        double finishOrientingThreshold, smoothAngularError, alpha, errorInt, lengthAddition;
        // Segment indices are 1-indexed
        int outerSegIndex, innerSegIndex, updateFrequency, numSegments, closeSegIndex;
        std::vector<LineSegment> path;
        std::vector<geometry_msgs::Vector3> activeSegIntersections;
        std::vector<geometry_msgs::Vector3> innerSegIntersections;
        std::vector<geometry_msgs::Vector3> nextSegIntersections;
        geometry_msgs::Vector3 finalPoint;
        geometry_msgs::Vector3 targetPoint;
        geometry_msgs::Vector3 currentPos;
        geometry_msgs::Pose pathPoints[2];
        ros::Subscriber localisationSub;
        ros::Publisher motorControllerPub;
        ros::Publisher robotMarkerPub;
        ros::Publisher pathMarkerPub;
        ros::Publisher targetPub;
        // DEBUG: Calibration
        ros::Publisher smoothedErrorPub;
        ros::Publisher errorPub;
        double prevError;

        visualization_msgs::Marker robotMarker;
        visualization_msgs::Marker pathMarker;
        visualization_msgs::Marker targetMarker;
        void localisationCallback(const geometry_msgs::Vector3::ConstPtr& msg);
        void createPath(const pure_pursuit_controller::PurePursuitGoalConstPtr &goal);
        void updateTargetPoint();
        void publishControlInput(double desiredOrientation = 0.0);
        int getCloseSegIndex();
        //geometry_msgs::Vector3 lineStartPoint;
        //geometry_msgs::Vector3 lineEndPoint;
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<pure_pursuit_controller::PurePursuitAction> as_;
        std::string action_name_;
        pure_pursuit_controller::PurePursuitFeedback feedback_;
        pure_pursuit_controller::PurePursuitResult result_;

    public:

        PurePursuitAction(std::string name);
        // TODO: Delete outdated line-segments!!!! So like at the end of function?
        void executeCB(const pure_pursuit_controller::PurePursuitGoalConstPtr &goal);
};

PurePursuitAction::PurePursuitAction(std::string name) :
    as_(nh_, name, boost::bind(&PurePursuitAction::executeCB, this, _1), false),
    action_name_(name)
{
    // TODO: Initiallise current position to weird value so that we can see if it it has received data?
    as_.start();
    nh_.getParam("r", r);
    nh_.getParam("rInner", rInner);
    nh_.getParam("stopDistance", stopDistance);
    nh_.getParam("switchToDriveThreshold", switchToDriveThreshold);
    nh_.getParam("switchToTurnThreshold", switchToTurnThreshold);
    nh_.getParam("finishOrientingThreshold", finishOrientingThreshold);
    nh_.getParam("desiredV", desiredV);
    maxDesV = desiredV;
    nh_.getParam("Kp", Kp);
    nh_.getParam("Ki", Ki);
    nh_.getParam("Kd", Kd);
    nh_.getParam("alpha", alpha);
    nh_.getParam("closeToGoalDist", closeToGoalDist);
    nh_.getParam("lengthAddition", lengthAddition);
    nh_.getParam("/params/updateFrequency", updateFrequency);
    dt = 1/updateFrequency;
    outerSegIndex = 0; // Index of active segment, starts at 1 (0 means not set)
    unconstrainedErrorAddition = 0.0;
    smoothAngularError = 0.0;
    smoothAngularErrorOld = 0.0;                        //localisation/position
    localisationSub = nh_.subscribe<geometry_msgs::Vector3>("/localisation/position", 1, &PurePursuitAction::localisationCallback, this);
    motorControllerPub = nh_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    robotMarkerPub = nh_.advertise<visualization_msgs::Marker>("robot_marker", 1);
    pathMarkerPub = nh_.advertise<visualization_msgs::Marker>("path_marker", 1);
    targetPub = nh_.advertise<visualization_msgs::Marker>("target_marker", 1);
    // DEBUG: Calibration
    smoothedErrorPub = nh_.advertise<std_msgs::Float32>("smoothError", 1);
    errorPub = nh_.advertise<std_msgs::Float32>("error", 1);

    // Setting up markers
    robotMarker.header.frame_id = "/base_map"; pathMarker.header.frame_id = "/base_map"; targetMarker.header.frame_id = "/base_map";
    robotMarker.header.stamp = ros::Time::now(); pathMarker.header.stamp = ros::Time::now(); targetMarker.header.stamp = ros::Time::now();
    robotMarker.ns = "robot_marker"; pathMarker.ns = "path_marker"; targetMarker.ns ="targer_marker";
    robotMarker.id = 0; pathMarker.id = 0; targetMarker.id = 0;

    robotMarker.type = visualization_msgs::Marker::SPHERE;
    robotMarker.action = visualization_msgs::Marker::ADD;
    robotMarker.pose.position.x = 0; robotMarker.pose.position.y = 0; robotMarker.pose.position.z = 0;
    robotMarker.pose.orientation.x = 0.0; robotMarker.pose.orientation.y = 0.0; robotMarker.pose.orientation.z = 0.0; robotMarker.pose.orientation.w = 1.0;
    robotMarker.scale.x = 2*r; robotMarker.scale.y = 2*r; robotMarker.scale.z = 2*r;
    robotMarker.color.r = 0.0f; robotMarker.color.g = 0.0f; robotMarker.color.b = 1.0f; robotMarker.color.a = 0.5;
    robotMarker.lifetime = ros::Duration();

    pathMarker.type = visualization_msgs::Marker::LINE_STRIP;
    pathMarker.action = visualization_msgs::Marker::ADD;
    pathMarker.color.r = 0.0f; pathMarker.color.g = 1.0f; pathMarker.color.b = 0.0f; pathMarker.color.a = 1.0;
    pathMarker.scale.x = 0.05;
    pathMarker.lifetime = ros::Duration();

    targetMarker.type = visualization_msgs::Marker::SPHERE;
    targetMarker.action = visualization_msgs::Marker::ADD;
    targetMarker.pose.position.x = 0; targetMarker.pose.position.y = 0; targetMarker.pose.position.z = 0;
    targetMarker.pose.orientation.x = 0.0; targetMarker.pose.orientation.y = 0.0; targetMarker.pose.orientation.z = 0.0; targetMarker.pose.orientation.w = 1.0;
    targetMarker.scale.x = 0.05; targetMarker.scale.y = 0.05; targetMarker.scale.z = 0.05;
    targetMarker.color.r = 1.0f; targetMarker.color.g = 1.0f; targetMarker.color.b = 0.0f; targetMarker.color.a = 1.0;
    targetMarker.lifetime = ros::Duration();

    state = State::TURN;
}

void PurePursuitAction::executeCB(const pure_pursuit_controller::PurePursuitGoalConstPtr &goal)
{

  state = State::TURN;
  ROS_INFO("Got into the call at least");
  // Helper variables
  ros::Rate r(updateFrequency);
  bool success = true;
  bool shouldContinue = true;
  numSegments = goal->intermediatePoints.poses.size()-1;

  // TODO: Only for test
  geometry_msgs::Vector3 pose;
  if (numSegments > 0)
  {
    finalPoint.x = goal->intermediatePoints.poses[numSegments].pose.position.x;
    finalPoint.y = goal->intermediatePoints.poses[numSegments].pose.position.y;
    finalPoint.z = goal->intermediatePoints.poses[numSegments].pose.position.z;
    createPath(goal);
    // Index of segment that counts as close to goal. Send feedback when this segment is reached
    closeSegIndex = getCloseSegIndex();
    feedback_.isCloseToGoal = false;
  }
  else
  {
    state = State::ORIENTING;
  }

  //ros::spinOnce();
  /*lineStartPoint.x = currentPos.x;
  lineStartPoint.y = currentPos.y;
  lineStartPoint.z = 0.1f;*/
  //lineEndPoint.x = goal->intermediatePoints.poses[0].pose.position.x;
  //lineEndPoint.y = goal->intermediatePoints.poses[0].pose.position.y;
  //lineEndPoint.z = 0.1f;
  //ROS_INFO("lineEndPoint: %f %f", lineEndPoint.x, lineEndPoint.y);
  //std::vector<geometry_msgs::Vector3> pointsForPathMarker(2); // TODO: REMOVE

  ROS_INFO("%s, Executing, testing pure pursuit server", action_name_.c_str());

  // TODO: Add feedback_!
  outerSegIndex = 1;
  innerSegIndex = 1;
  // Initial error is reasonably zero, so initial desired heading should be equal to the initial heading
  desiredHeadingOld = currentPos.z;
  while (state != State::FINISHED)
  {
    //pathMarkerPub.publish(pathMarker);
    ros::spinOnce();
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
      break;
    }
    if (state != State::ORIENTING)
    {
        updateTargetPoint();
    }
    publishControlInput(goal->intermediatePoints.poses[0].pose.orientation.z);
    r.sleep();
  }

  if(success)
  {
    // TODO: Change
    pose = currentPos;
    result_.pose = pose;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);

    geometry_msgs::Twist desiredTwist;
    desiredTwist.linear.x = 0.0;
    desiredTwist.angular.z = 0.0;
    motorControllerPub.publish(desiredTwist);
  }
  // According to https://stackoverflow.com/questions/10464992/c-delete-vector-objects-free-memory#10465032
  // this line clear and deallocates memory taken up by path
  std::vector<LineSegment>().swap(path);
}

void PurePursuitAction::createPath(const pure_pursuit_controller::PurePursuitGoalConstPtr &goal)
{
  ROS_INFO("Into path creation");
  geometry_msgs::Point startPoint, endPoint;

  for(int i=1; i<=numSegments; i++)
  {
    geometry_msgs::Vector3 startVector, endVector;
    startVector.x = goal->intermediatePoints.poses[i-1].pose.position.x;
    startVector.y = goal->intermediatePoints.poses[i-1].pose.position.y;
    startPoint.x = goal->intermediatePoints.poses[i-1].pose.position.x;
    startPoint.y = goal->intermediatePoints.poses[i-1].pose.position.y;
    endVector.x = goal->intermediatePoints.poses[i].pose.position.x;
    endVector.y = goal->intermediatePoints.poses[i].pose.position.y;
    endPoint.x = goal->intermediatePoints.poses[i].pose.position.x;
    endPoint.y = goal->intermediatePoints.poses[i].pose.position.y;
    this->path.push_back(LineSegment(startVector, endVector, lengthAddition));
    this->pathMarker.points.push_back(startPoint);
  }
  this->pathMarker.points.push_back(endPoint);
}

void PurePursuitAction::updateTargetPoint()
{
  activeSegIntersections = path[outerSegIndex-1].getIntersectionPoints(currentPos.x, currentPos.y, r);
  innerSegIntersections = path[innerSegIndex-1].getIntersectionPoints(currentPos.x, currentPos.y, rInner);
  bool checkNextSegs = false;

  //ROS_INFO("Num of intersects with inner: %d", innerSegIntersections.size());

  // -------- Finds index of segment intersecting with inner circle --------
  int tempInnerSegIndex = innerSegIndex;
  while (tempInnerSegIndex <= numSegments)
  {
    if (innerSegIntersections.size() == 2)
    {
        innerSegIndex = tempInnerSegIndex;
        break;
    }
    else if (innerSegIntersections.size() == 1)
    {
      if (innerSegIntersections[0].z > 0)
      {
        // The back part of robot circle is not intersecting with anything,
        // but front part is. We know since this info is stored in z-coordinate
        innerSegIndex = tempInnerSegIndex;
        break;
      }
    }
    ++tempInnerSegIndex;
    innerSegIntersections = path[tempInnerSegIndex-1].getIntersectionPoints(currentPos.x, currentPos.y, rInner);
    //ROS_INFO("Num of intersects with inner: %d", innerSegIntersections.size());
  }

  // -------- Finds target point based on intersection with outer circle ---------
  if (activeSegIntersections.size() == 2)
  {
    targetPoint = activeSegIntersections[1];
    //ROS_INFO("Current intersection: (%f, %f)", targetPoint.x, targetPoint.y);
  }
  else if (activeSegIntersections.size() == 1)
  {
    if (activeSegIntersections[0].z > 0)
    {
      // The back part of robot circle is not intersecting with anything,
      // but front part is. This info is stored in z-coordinate
      targetPoint = activeSegIntersections[0];
      //ROS_INFO("Weird current intersection: (%f, %f): ", targetPoint.x, targetPoint.y);
    }
    else if (outerSegIndex == numSegments)
    {
      targetPoint = finalPoint;
      //ROS_INFO("Final intersection: (%f, %f): ", targetPoint.x, targetPoint.y);
    }
    else
    {
      // Only back of circle intersects with current segment, front of circle
      // must be intersecting something else
      checkNextSegs = true;
    }
  }
  else
  {
    checkNextSegs = true;
  }

  // -------- Didn't find target point on current segment, check next segments ----------
  if (checkNextSegs && outerSegIndex < numSegments)
  {
    int tempOuterSegIndex = outerSegIndex;
    do
    {
      ++tempOuterSegIndex;
      activeSegIntersections = path[tempOuterSegIndex-1].getIntersectionPoints(currentPos.x, currentPos.y, r);
    }
    while (activeSegIntersections.size() == 0 && tempOuterSegIndex < numSegments);
    // After introduction of inner radius, it seems that sometimes no intersections with outer radius are found.
    // In such a case, we don't want to update the outerSegIndex
    if (activeSegIntersections.size() > 0)
    {
      outerSegIndex = tempOuterSegIndex;
    }

    if (activeSegIntersections.size() > 0)
    {
      targetPoint = activeSegIntersections[activeSegIntersections.size()-1];
    }
    else {
      // TODO: If distance to final point is smaller than radius, go to it
      // If it's greater, return error and ask for new path
      ROS_INFO("WARNING: Could not find intersection with next segment, heading to final point");
      targetPoint = finalPoint;
    }

  }

  // --------- Possibly adapts target point to avoid cutting corners ---------
  desiredV = maxDesV;
  /*if (outerSegIndex - innerSegIndex > 0)
  {
    targetPoint = path[innerSegIndex-1].getEndPoint();
    //desiredV = 0.1;
  }*/
  //ROS_INFO("Next intersection: (%f, %f)", targetPoint.x, targetPoint.y);

  targetMarker.pose.position.x = targetPoint.x;
  targetMarker.pose.position.y = targetPoint.y;
  targetPub.publish(targetMarker);
  if (innerSegIndex >= closeSegIndex && !feedback_.isCloseToGoal)
  {
    feedback_.isCloseToGoal = true;
    as_.publishFeedback(feedback_);
  }
  //ROS_INFO("Current target: (%f, %f)", targetPoint.x, targetPoint.y);//DEBUG

  ROS_INFO("Outer index: %d, inner index: %d", outerSegIndex, innerSegIndex);

  double currentDistanceSqr = (finalPoint.x - currentPos.x)*(finalPoint.x - currentPos.x) + (finalPoint.y - currentPos.y)*(finalPoint.y - currentPos.y);
  //if (outerSegIndex == numSegments && currentDistanceSqr < stopDistance*stopDistance)
  if (currentDistanceSqr < stopDistance*stopDistance)
  {
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    state = State::FINISHED;
  }

}

void PurePursuitAction::publishControlInput(double desiredOrientation)
{
  // ----------------- Sets variable values -----------------
  double currentHeadingUnconstrained = currentPos.z;
  // Normalises currentHeading to [0, 2pi]
  double currentHeading = fmod(currentHeadingUnconstrained, 2*M_PI);
  if (currentHeading < 0)
  {
    currentHeading += 2*M_PI;
  }

  double desiredHeading;
  if (state != State::ORIENTING)
  {
    geometry_msgs::Vector3 diffVector = targetPoint;
    diffVector.x -= currentPos.x;
    diffVector.y -= currentPos.y;
    desiredHeading = atan2(diffVector.y, diffVector.x);
  }
  else
  {
    desiredHeading = desiredOrientation;
  }


  float angularError = desiredHeading - currentHeading;
  // angularError is currently in the range [-3*PI, PI], we map it into the range [-PI, PI]
  if (angularError < -M_PI)
  {
    angularError += 2*M_PI;
  }
  errorInt += angularError;

  // desiredHeading is kept in the range [-pi, pi], making it discontinuous
  // To ensure that angularErrorUnconstrained retains its continuity despite this,
  // the jumps in value of desiredHeading are cancelled out by unconstrainedErrorAddition.
  // angularErrorUnconstrained must be continuous for us to correctly approximate derivative
  if (desiredHeading - desiredHeadingOld > M_PI)
  {
    unconstrainedErrorAddition -= 2*M_PI;
  }
  else if (desiredHeading - desiredHeadingOld < -M_PI)
  {
    unconstrainedErrorAddition += 2*M_PI;
  }
  double angularErrorUnconstrained = desiredHeading - currentHeadingUnconstrained + unconstrainedErrorAddition;
  smoothAngularErrorOld = smoothAngularError;
  smoothAngularError = alpha*angularErrorUnconstrained + (1 - alpha)*smoothAngularErrorOld;

  // ------------- Checks state ---------------
  if (fabs(angularError) < switchToDriveThreshold && state == State::TURN)
  {
    state = State::DRIVE;
  }
  else if (fabs(angularError) > switchToTurnThreshold && state != State::ORIENTING)
  {
    state = State::TURN;
  }
  else if (state == State::ORIENTING && fabs(angularError) < finishOrientingThreshold)
  {
    state = State::FINISHED;
  }
  // ------------ Calculates output -------------
  geometry_msgs::Twist desiredTwist;
  if (state == State::DRIVE)
  {
    // DEBUG: Calibration
    nh_.getParam("Kp", Kp);
    nh_.getParam("Ki", Ki);
    nh_.getParam("Kd", Kd);
    //nh_.getParam("desiredHeading", desiredHeading);

    desiredTwist.linear.x = desiredV;
  }
  else if (state == State::TURN || state == State::ORIENTING){
    //DEBUG Calibration
    Kp = 0.3;
    Ki = 0.0;
    Kd = 0.0;

    desiredTwist.linear.x = 0.0;
  }
  else
  {
    ROS_INFO("WHYYYYAYAYYAYAYAYYAYAYAYAYYAYAYAYAYAYYAYAYA");
  }
  desiredTwist.angular.z = Kp*angularError + Ki*errorInt + Kd*(smoothAngularError - smoothAngularErrorOld);
  if (state == State::TURN)
  {
    double sign;
    if (angularError > 0.0) {
      sign = 1;
    }
    else {
      sign = -1.0;
    }
    desiredTwist.angular.z = 0.8*sign;
  }
  motorControllerPub.publish(desiredTwist);
  desiredHeadingOld = desiredHeading;
  prevError = angularErrorUnconstrained;

  // ---------------------- DEBUG ------------------------
  // DEBUG: Rviz
  robotMarker.pose.position.x = currentPos.x;
  robotMarker.pose.position.y = currentPos.y;
  robotMarkerPub.publish(robotMarker);

  std_msgs::Float32 errorMsg;
  std_msgs::Float32 smoothErrorMsg;
  errorMsg.data = angularErrorUnconstrained - prevError;
  smoothErrorMsg.data = smoothAngularError-smoothAngularErrorOld;
  smoothedErrorPub.publish(smoothErrorMsg);
  errorPub.publish(errorMsg);
  //ROS_INFO("ERROR: %f   |   SMOOTH: %f", angularErrorUnconstrained - prevError, smoothAngularError-smoothAngularErrorOld);
  //ROS_INFO("AngleUn: %f, angluarErrUn: %f, prev: %f", currentHeadingUnconstrained, angularErrorUnconstrained, prevError);

  //DEBUG CALIBRATION
  //Kp = 0.5;
  //Ki = 0;
  //Kd = 0;

  // DEBUG
  double errorDeg = 180*angularError/M_PI;
  double desiredDeg = 180*desiredHeading/M_PI;
  double currentDeg = 180*currentHeading/M_PI;
  //ROS_INFO("Kp: %f, Kd: %f, Ki: %f", Kp, Kd, Ki);
  //ROS_INFO("Current error is: %f, desiredHeading: %f, currentHeading: %f", errorDeg, desiredDeg, currentDeg);
}

int PurePursuitAction::getCloseSegIndex()
{
  double distFromEnd = 0;
  for (int i = numSegments-1; i >= 0; i--)
  {
    distFromEnd += path[i].segLen();
    if (distFromEnd >= closeToGoalDist)
    {
      return i+1;
    }
  }
  return 1;
}

void PurePursuitAction::localisationCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  currentPos = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "purePursuit");

  //TODO: Remove
  ROS_INFO("STARTED");

  PurePursuitAction purePursuit("purePursuit");
  ros::spin();  // TODO: Does this mean that we don't need the spinOnce() in executeCB()?

  // DEBUG
  /*geometry_msgs::Vector3 startPose;
  geometry_msgs::Vector3 endPose;
  startPose.x = 0;
  startPose.y = 0;
  endPose.x = 4;
  endPose.y = 4;
  LineSegment segment = LineSegment(startPose, endPose);
  std::vector<geometry_msgs::Vector3> intersectionPoints = segment.getIntersectionPoints(1.0, 3.0, 2.0);
  ROS_INFO("size: %d", intersectionPoints.size());
  ROS_INFO("(%f, %f)", intersectionPoints[0].x, intersectionPoints[0].y);
  ROS_INFO("(%f, %f)", intersectionPoints[1].x, intersectionPoints[1].y);*/
  //new LineSegment(startPose, endPose);
  //LineSegment *segment = new LineSegment(startPose, endPose);

  return 0;
}
