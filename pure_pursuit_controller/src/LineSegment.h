#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/Vector3.h"

class LineSegment {
  private:
    geometry_msgs::Vector3 startingPoint;
    geometry_msgs::Vector3 endPoint;
    geometry_msgs::Vector3 directionVector;
    double arg;   // Argument of direction vector in range [-PI, PI]
    double length;
    double lengthAddition;
    double pTerm(double xc, double yc);
    double qTerm(double xc, double yc, double r);

  public:
    LineSegment(geometry_msgs::Vector3 startingPoint, geometry_msgs::Vector3 endPoint, double lengthAddition);
    std::vector<geometry_msgs::Vector3> getIntersectionPoints(double xc, double yc, double r);
    geometry_msgs::Vector3 getDirectionVector();
    geometry_msgs::Vector3 getEndPoint();
    double getArg();
    double segLen();

};
