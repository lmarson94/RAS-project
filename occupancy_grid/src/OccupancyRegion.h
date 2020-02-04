#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class OccupancyRegion
{
  public:
    OccupancyRegion(nav_msgs::OccupancyGrid* sourceGrid, int regionWidth, int regionHeight, int gridWidth, int startX, int startY, int unexploredBoundMin = -1, int unexploredBoundMax = 50);
    int getFurthestUnexploredIndex(int x, int y);
    bool isExplored();
    double midPoint[2];
    // DEBUG: Remove after test
    //void addCost(const int startX, const int startY, const int width, const int height, const int cost);
  private:
    nav_msgs::OccupancyGrid* sourceGrid; // Move to private
    int regionHeight, regionWidth, gridWidth, gridHeight, startX, startY, unexploredBoundMin, unexploredBoundMax;
    bool regionIsExplored;
};
