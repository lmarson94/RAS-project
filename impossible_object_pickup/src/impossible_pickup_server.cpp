#include "impossible_pickup_server.h"
/*
#include "ros/ros.h"
#include "impossible_object_pickup/PickupImpossibleObject.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include <tuple>
*/
/*
//typedef std::tuple<int, int> i2tuple;

struct SegmentInfo {
	double occupiedLengthSqr;
	double freeLengthSqr;
	geometry_msgs::Pose furthestFreePoint;
};

double getVectorLengthSqr(geometry_msgs::Pose startPose, geometry_msgs::Pose endPose);*/

nav_msgs::OccupancyGrid grid;
int n_width, n_height;
double resolution;
bool hasMap = false;
int minOccupiedCost = 50;	// TODO: Use some other place to get this info?

// -------------------------------------------------------------------------------------------
// TODO: WHEN YOU CONVERT CELL TO POSITION, DO YOU MAKE POSITION BE IN THE MIDDLE OF CELL?????
// -------------------------------------------------------------------------------------------

bool getPoints( impossible_object_pickup::PickupImpossibleObject::Request &req,
		impossible_object_pickup::PickupImpossibleObject::Response &res)
{
	 geometry_msgs::Point potentialPoint1;
	 i2tuple objCell = pointToCell(req.object_pos);
	 i2tuple closestFreeCell = getClosestFreeCell(objCell, minOccupiedCost);
	 // DEBUG
	 //geometry_msgs::Pose objPos = cellToPose(objCell);
	 //geometry_msgs::Pose closePos = cellToPose(closestFreeCell);
	 //ROS_INFO("Obj cell: (%d, %d) | close cell: (%d, %d)", std::get<0>(objCell), std::get<1>(objCell), std::get<0>(closestFreeCell), std::get<1>(closestFreeCell));
	 //ROS_INFO("Org obj pos: (%f, %f)", req.object_pos.position.x, req.object_pos.position.y);
	 //ROS_INFO("Obj pos: (%f, %f) | close pos: (%f, %f)", objPos.position.x, objPos.position.y, closePos.position.x, closePos.position.y);
	 geometry_msgs::Vector3 freedomVector = getUnitVector(req.object_pos, cellToPoint(closestFreeCell));
	 potentialPoint1 = getPointInDirection(req.object_pos, freedomVector, 0.3);		// TODO: FIGURE OUT APPROPRIATE DISTANCE
	 std::vector<i2tuple> line = getLine(closestFreeCell, pointToCell(potentialPoint1));

	 // TODO: Is cost correct????
	 if (areAllFree(line, minOccupiedCost-1))
	 {
		 res.point1 = potentialPoint1;
		 res.point2 = cellToPoint(closestFreeCell);
		 return true;
	 }

	 int numAngleIncrements = 18;
	 double angleIncrement = M_PI/(2*numAngleIncrements);
	 geometry_msgs::Vector3 alternativeVector;
	 for (int i = 1; i <= numAngleIncrements; ++i)
	 {
		 for (int mult = -1; mult <= 1; mult += 2)
		 {
			 alternativeVector = rotateVector(freedomVector, mult*i*angleIncrement);
	  	 potentialPoint1 = getPointInDirection(req.object_pos, alternativeVector, 0.35);// TODO: avoid hard-coded distance
	  	 line = getLine(objCell, pointToCell(potentialPoint1));
	  	 SegmentInfo segInfo = getSegmentInfo(line, 49);

			 //ROS_INFO("Actual: %f, desired: %f", segInfo.approachableLengthSqr, 0.28*0.28);
	  	 if (segInfo.approachableLengthSqr >= 0.28*0.28)
	  	 {
	  		 res.point1 = segInfo.furthestFreePoint;
	  		 res.point2 = segInfo.closestFreePoint;
	  		 return true;
	  	 }
		 }
	 }
	 return false;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	grid = *msg;
	n_width = grid.info.width;
	n_height = grid.info.height;
	resolution = grid.info.resolution;
	hasMap = true;
}

bool inBounds(int x, int y)
{
  if( (x >= 0 && x < n_width) && (y >= 0 && y < n_height))
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<i2tuple> getLine(i2tuple start, i2tuple end)
{
  std::vector<i2tuple> traversed;

  int start_x, start_y, end_x, end_y;
  double dx, dy;

  std::tie(start_x,start_y) = start;
  std::tie(end_x,end_y) = end;

  int x = start_x;
  int y = start_y;

  std::tuple<float,float> dxdy = std::make_tuple(fabs(end_x - start_x),fabs(end_y - start_y));
  std::tie(dx,dy) = dxdy;
  float n = dx + dy;
  int x_inc = 1;

  if (end_x <= start_x)
  {
    x_inc = -1;
  }
  int y_inc = 1;
  if (end_y <= start_y)
  {
    y_inc = -1;
  }
  float error = dx - dy;
  dx = dx * 2;
  dy = dy * 2;

  for (int i = 0; i < (int) n; i++) {
    traversed.push_back(i2tuple(x,y));

    if (error > 0)
    {
      x = x + x_inc;
      error = error - dy;
    }
    else
    {
      if (error == 0)
      {
        traversed.push_back(i2tuple(x+x_inc,y));
      }
      y = y + y_inc;
      error = error + dx;
    }
  }
  traversed.push_back(i2tuple(end_x,end_y));
  return traversed;
}

bool areAllFree(std::vector<i2tuple> line, int maxFreeCost)
{
		bool areFree = true;
		for (std::vector<i2tuple>::size_type i = 0; i != line.size(); i++)
		{
			int x, y;
			std::tie(x, y) = line[i];
			if (grid.data[y*n_width + x] > maxFreeCost)
			{
				areFree = false;
				break;
			}
		}
		return areFree;
}

geometry_msgs::Point cellToPoint(i2tuple cell)
{
	// TODO: Check that this is actually correct
	int xIndex, yIndex;
	std::tie(xIndex, yIndex) = cell;
	double xCoord = xIndex*resolution + 0.5*resolution;
	double yCoord = yIndex*resolution + 0.5*resolution;
	geometry_msgs::Point point;
	point.x = xCoord;
	point.y = yCoord;
	return point;
}

i2tuple pointToCell(geometry_msgs::Point point)
{
	int xIndex, yIndex;
	double temp = ( (int)(point.x/resolution) ) *resolution + 0.5*resolution;
	//ROS_INFO("Crazy: %f, original: %f", temp, pose.position.x);
	xIndex = (int)(point.x/resolution);
	yIndex = (int)(point.y/resolution);
	return std::make_tuple(xIndex, yIndex);
}

SegmentInfo getSegmentInfo(std::vector<i2tuple> line, int maxFreeCost)
{
	int firstFreeIndex = -1;
	int furthestFreePointIndex = -1;
	for (std::vector<i2tuple>::size_type i = 0; i != line.size(); i++)
	{
		int x, y;
		std::tie(x, y) = line[i];
		if (grid.data[y*n_width + x] > maxFreeCost)
		{
			if (firstFreeIndex >= 0)
			{
				// The line has been free but now became occupied
				furthestFreePointIndex = i - 1;
				break;
			}
		}
		else
		{
			if (firstFreeIndex < 0)
			{
				firstFreeIndex = i;
			}
		}
	}
	if (furthestFreePointIndex == -1) {
		furthestFreePointIndex = line.size()-1;
	}
	if (firstFreeIndex == -1) {
		firstFreeIndex = 0;
		furthestFreePointIndex = 0;
	}
	//ROS_INFO("firstFreeIndex: %d, furthestFreePointIndex: %d", firstFreeIndex, furthestFreePointIndex);
	geometry_msgs::Point furthestFreePoint = cellToPoint(line[furthestFreePointIndex]);
	geometry_msgs::Point closestFreePoint = cellToPoint(line[firstFreeIndex]);
	double occupiedLengthSqr = getVectorLengthSqr(cellToPoint(line[0]), closestFreePoint);
	double freeLengthSqr = getVectorLengthSqr(closestFreePoint, furthestFreePoint);
	double approachableLengthSqr = getVectorLengthSqr(cellToPoint(line[0]), furthestFreePoint);
	SegmentInfo segInfo;
	segInfo.occupiedLengthSqr = occupiedLengthSqr;
	segInfo.freeLengthSqr = freeLengthSqr;
	segInfo.furthestFreePoint = furthestFreePoint;
	segInfo.closestFreePoint = closestFreePoint;
	segInfo.approachableLengthSqr = approachableLengthSqr;
	return segInfo;
}

double getVectorLengthSqr(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint)
{
	return pow(startPoint.x - endPoint.x, 2) + pow(startPoint.y - endPoint.y, 2);
}

i2tuple getClosestFreeCell(i2tuple objCell, int minOccupiedCost)
{
	int objX, objY;
	std::tie(objX, objY) = objCell;
	int R = 1;
	bool hasFoundCell = false;
	i2tuple unoccupiedCell;
	while (!hasFoundCell)
	{
		int Y = R;
		for (int y = -Y; y <= Y; y++)
		{
			int X = (int)(sqrt(pow(R,2) - pow(y,2)));
			for (int x = -X; x <= X; x++)
			{
				if (inBounds(objX+x, objY+y) && grid.data[(y+objY)*n_width + x+objX] < minOccupiedCost)
				{
					hasFoundCell = true;
					unoccupiedCell = std::make_tuple(x+objX, y+objY);
					break;
				}
			}
		}
		R++;
	}
	return unoccupiedCell;
}

geometry_msgs::Vector3 getUnitVector(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
	geometry_msgs::Vector3 returnVector;
	double vectorLen = sqrt(getVectorLengthSqr(point1, point2));
	returnVector.x = (point2.x - point1.x)/vectorLen;
	returnVector.y = (point2.y - point1.y)/vectorLen;
	return returnVector;
}

geometry_msgs::Vector3 rotateVector(geometry_msgs::Vector3 vec, double angle)
{
	geometry_msgs::Vector3 returnVector;
	returnVector.x = cos(angle)*vec.x - sin(angle)*vec.y;
	returnVector.y = sin(angle)*vec.x + cos(angle)*vec.y;
	return returnVector;
}

// dir is assumed to be a unit vector!!!
geometry_msgs::Point getPointInDirection(geometry_msgs::Point point1, geometry_msgs::Vector3 dir, double len)
{
	geometry_msgs::Point point2;
	point2.x = point1.x + len*dir.x;
	point2.y = point1.y + len*dir.y;
	return point2;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "impossible_pickup_server");
  ros::NodeHandle n;
	ros::Subscriber mapSub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
	while (!hasMap)
	{
		ros::spinOnce();
	}

  ros::ServiceServer service = n.advertiseService("pickup_impossible_object", getPoints);
  ROS_INFO("Ready to pickup up impossible point");
  ros::spin();

  return 0;
}
