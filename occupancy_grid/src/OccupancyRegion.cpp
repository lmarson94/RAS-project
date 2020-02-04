#include <ros/ros.h>
#include "OccupancyRegion.h"
#include <iostream> // TODO: Remove
using namespace std;

OccupancyRegion::OccupancyRegion(nav_msgs::OccupancyGrid* sourceGrid, int regionWidth, int regionHeight, int gridWidth, int startX, int startY, int unexploredBoundMin, int unexploredBoundMax)
{
  this->sourceGrid = sourceGrid;
  this->regionWidth = regionWidth;
  this->regionHeight = regionHeight;
  this->gridWidth = gridWidth;
  this->gridHeight = (int) sourceGrid->data.size()/gridWidth;
  this->startX = startX;
  this->startY = startY;
  this->unexploredBoundMin = unexploredBoundMin; // Lowest value counting as unexplored
  this->unexploredBoundMax = unexploredBoundMax; // Highest value counting as unexplored
  this->regionIsExplored = false;
  this->midPoint[0] = startX + 0.5*regionWidth;
  this->midPoint[1] = startY + 0.5*regionHeight;
}

// Returns -1 if no unexplored cells were found
int OccupancyRegion::getFurthestUnexploredIndex(int x, int y)
{
  if (regionIsExplored) return -1;
  int optIndex = -1;
  double optDistanceSqr = 0;
  for (int i=0; i < regionHeight; i++)
  {
    for (int j=0; j < regionWidth; j++)
    {
      double distanceSqr = (x-startX-j)*(x-startX-j) + (y-startY-i)*(y-startY-i);
      if (sourceGrid->data[(startY+i)*gridWidth + j + startX] < unexploredBoundMax && sourceGrid->data[(startY+i)*gridWidth + j + startX] > unexploredBoundMin && distanceSqr > optDistanceSqr)
      {
        optIndex = (startY+i)*gridWidth + startX + j;
        optDistanceSqr = distanceSqr;
      }
    }
  }

  if (optIndex == -1)
  {
    regionIsExplored = true;
  }

  return optIndex;
}

bool OccupancyRegion::isExplored()
{
  return this->regionIsExplored;
}

// DEBUG: Remove after test
/*void OccupancyRegion::addCost(const int startX, const int startY, const int width, const int height, const int cost)
{
    //ROS_INFO("YEAH TOTALLY ADDING COST OVER HERE! TO OBJECT:");
    //std::cout << sourceGrid << std::endl;
    for (int i=0; i < height; i++)
    {
      for (int j=0; j < width; j++)
      {
        if (sourceGrid->data[ (startY + i)*gridWidth + (j+startX)] < cost)
        {
          sourceGrid->data[ (startY + i)*gridWidth + (j+startX)] = cost;
        }
      }
    }
}
*/
