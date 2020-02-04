#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <occupancy_grid/line_msg.h>
#include <occupancy_grid/point_msg.h>
#include <occupancy_grid/points_msg.h>

class Mapping
{


  public:
    void main(int argc, char **argv);
    Mapping();
    struct wall {
      float m;
      float q;
      float x1, x2;
      float y1, y2;
    };
    struct point {
      float x;
      float y;
    };
    typedef std::vector<std::vector<int>> matrix;

  private:
    const int MAX_FREQUENCY = 15;
    const int WINDOW_SIZE = 5;
    const float THRESHOLD_LIKELIHOOD = log(1 + 0.45);
    const float THRESHOLD_DISTANCE = 0.7;

    int map_width, map_height;
    float x1, x2, y1, y2;
    float deltaX, deltaY, deltaTheta;
    float resolution;

    std::ifstream map_fs;
    std::string line, _map_file;

    /* Msgs */
    geometry_msgs::Vector3 position;
    nav_msgs::OccupancyGrid grid;
    matrix frequency;

    float angleIncrement;
    std::vector<float> laser;
    std::vector<wall> wallSet;

    /*Publishers*/
    ros::Publisher pub_grid, pub_lines, pub_line, pub_points;

    /*Subscribers*/
    ros::Subscriber sub_grid, sub_laser, sub_position, sub_odom;

    /* Functions */
    bool areEqual(float x, float y);
    bool smallerOrEqual(float x, float y);
    bool greaterOrEqual(float x, float y);
    float min(float x1, float x2);
    float max(float x1, float x2);

    void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scanCallback (const sensor_msgs::LaserScanConstPtr& msg);
    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void odometryCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    void mapping(float xm, float ym, float thetam);
    void getWalls(std::string _map_file);
};
