#include "LineSegment.h"

LineSegment::LineSegment(geometry_msgs::Vector3 point1, geometry_msgs::Vector3 point2, double lengthAddition)
{
  // A hope these are deleted as scope ends!
  startingPoint = geometry_msgs::Vector3();
  endPoint = geometry_msgs::Vector3();
  startingPoint.x = point1.x;
  startingPoint.y = point1.y;
  endPoint.x = point2.x;
  endPoint.y = point2.y;

  length = sqrt( (endPoint.x - startingPoint.x)*(endPoint.x - startingPoint.x) + (endPoint.y - startingPoint.y)*(endPoint.y - startingPoint.y) );
  lengthAddition = lengthAddition;     // TODO: Avoid hardcoding
  directionVector = geometry_msgs::Vector3();
  directionVector.x = (endPoint.x - startingPoint.x)/length;
  directionVector.y = (endPoint.y - startingPoint.y)/length;
  arg = atan2(directionVector.y, directionVector.x);

}

// The z-coordinate of an intersection point represents whether the intersection
// is in front of the center or not. z==1 means in front of the center,
// z==-1 means behind the center
// If two intersections are found, the first is always the one behind the robot
std::vector<geometry_msgs::Vector3> LineSegment::getIntersectionPoints(double xc, double yc, double r)
{
  int numIntersections = 0;
  double p = pTerm(xc, yc);
  double q = qTerm(xc, yc, r);
  double t1, t2, t, z;

  if (0.25*p*p >= q)
  {
    numIntersections = 2;
    double squareRoot = sqrt(0.25*p*p - q);
    t1 = -0.5*p - squareRoot;
    t2 = -0.5*p + squareRoot;
    if (t1 < 0 || t1 > length + lengthAddition)
    {
      numIntersections -= 1;
      t = t2;
      z = 1.0;
    }
    if(t2 < 0 || t2 > length + lengthAddition)
    {
      numIntersections -= 1;
      t = t1;
      z = -1.0;
    }
  }

  if (numIntersections == 2)
  {
    geometry_msgs::Vector3 intersection1;
    geometry_msgs::Vector3 intersection2;
    intersection1.x = startingPoint.x + t1*directionVector.x;
    intersection1.y = startingPoint.y + t1*directionVector.y;
    intersection1.z = -1.0; // intersection behind center
    intersection2.x = startingPoint.x + t2*directionVector.x;
    intersection2.y = startingPoint.y + t2*directionVector.y;
    intersection2.z = 1.0;  // intersection in front of center
    std::vector<geometry_msgs::Vector3> intersectionPoints;
    intersectionPoints.push_back(intersection1);
    intersectionPoints.push_back(intersection2);

    return intersectionPoints;
  }
  else if (numIntersections == 1)
  {
    geometry_msgs::Vector3 intersection;
    intersection.x = startingPoint.x + t*directionVector.x;
    intersection.y = startingPoint.y + t*directionVector.y;
    intersection.z = z;
    std::vector<geometry_msgs::Vector3> intersectionPoints = {intersection};
    //std::vector<geometry_msgs::Vector3> intersectionPoints(1, intersection);

    return intersectionPoints;
  }
  else {
    std::vector<geometry_msgs::Vector3> intersectionPoints;

    return intersectionPoints;
  }

}

geometry_msgs::Vector3 LineSegment::getDirectionVector()
{
  return this->directionVector;
}

geometry_msgs::Vector3 LineSegment::getEndPoint()
{
  return this->endPoint;
}

double LineSegment::getArg()
{
  return this->arg;
}

double LineSegment::segLen()
{
  return this->length;
}

// Functions used for solving the quadratic equation.

double LineSegment::pTerm(double xc, double yc)
{
  return 2*( directionVector.x * (startingPoint.x - xc) + directionVector.y * (startingPoint.y - yc) );
}

double LineSegment::qTerm(double xc, double yc, double r)
{
  return (startingPoint.x - xc)*(startingPoint.x - xc) + (startingPoint.y - yc)*(startingPoint.y - yc) - r*r;
}
