#include "ros/ros.h"
#include "impossible_object_pickup/PickupImpossibleObject.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include <tuple>

typedef std::tuple<int, int> i2tuple;

struct SegmentInfo {
	double occupiedLengthSqr;
	double freeLengthSqr;
	double approachableLengthSqr;
	geometry_msgs::Point furthestFreePoint;
	geometry_msgs::Point closestFreePoint;
};

geometry_msgs::Vector3 rotateVector(geometry_msgs::Vector3 vec, double angle);

bool getPoints( impossible_object_pickup::PickupImpossibleObject::Request &req,
		impossible_object_pickup::PickupImpossibleObject::Response &res);

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

bool inBounds(int x, int y);

std::vector<i2tuple> getLine(i2tuple start, i2tuple end);

bool areAllFree(std::vector<i2tuple> line, int maxFreeCost);

geometry_msgs::Point cellToPoint(i2tuple cell);

i2tuple pointToCell(geometry_msgs::Point point1);

SegmentInfo getSegmentInfo(std::vector<i2tuple> line, int maxFreeCost);

double getVectorLengthSqr(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint);

i2tuple getClosestFreeCell(i2tuple objCell, int minOccupiedCost);

geometry_msgs::Vector3 getUnitVector(geometry_msgs::Point point11, geometry_msgs::Point point12);

// dir is assumed to be a unit vector!!!
geometry_msgs::Point getPointInDirection(geometry_msgs::Point point11, geometry_msgs::Vector3 dir, double len);
