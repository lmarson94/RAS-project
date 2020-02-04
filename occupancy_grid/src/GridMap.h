#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tuple>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <occupancy_grid/point_msg.h>
#include <occupancy_grid/points_msg.h>
#include <ras_object_master/ObjectMasterMsg.h>

using namespace std;

/************************************************
 *                                              *
 *              OccupancyGrid                   *
 *                                              *
 ************************************************/
class GridMap
{

  struct p{
    int x;
    int y;
  };

  typedef std::tuple<int, int> i2tuple;
  typedef std::tuple<double, double, double, double> i4tuple;
  // typedef std::vector<i4tuple> walls;
  public:
    GridMap(int n_width, int n_height, double resolution, string _map_file);
    nav_msgs::OccupancyGrid map;
    void inflateMap(const int R, const int cost);
    void inflateObstacle(const int R, const int cost, int x_obs, int y_obs);
    nav_msgs::OccupancyGrid getMap();
    void add_obstacle_to_map(double x, double y, double orientation_new);
    void add_point_to_map(const occupancy_grid::point_msg msg);
    void insert_points_to_map(const occupancy_grid::points_msg msg);
    void add_object_to_map(const geometry_msgs::Point msg);
    //void grid_size_callback(const geometry_msgs::Point::ConstPtr& msg);
  private:
    /******** Map properties ********/
    // const int width = 3;                // Width of grid [m]
    // const int height = 3;               // Height of grid [m]
    const int map_origin_x = 0;         // Origin x
    const int map_origin_y = 0;         // Origin y
    const int R1 = 15;                    // Inflation radius 1
    const int R2 = 21;                    // Inflation radius 2
    const int R1_obs = 3;
    const int R2_obs = 5;
    int n_width, n_height, n_elements;
    int occupied_space;
    double resolution;

    /* Coordinates to create wall*/
    double x1, y1, x2, y2, width, height, x_least, y_least;
    int x_max, y_max;

    /* New point */
    const int vertical = 2;
    const int horisontal = 3;

    /*Grid and vector*/
    std::vector<signed char> vec;       // 1D Vector of grid

    /* Strings */
    string frame_id, line;// _map_file;

    /* Geometry msgs */
    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion q;

    /* Navigation msgs */
    // nav_msgs::OccupancyGrid map;

    /* Map publisher*/
    ros::Publisher map_pub, map_bounds_pub;

    /* Point Subscriber */
    ros::Subscriber point_sub;

    /* Functions */
    std::vector<i2tuple> get_line(i2tuple start, i2tuple end);
    bool inBounds(int x, int y);
    void updateMap(i2tuple start, i2tuple end);
    void adjacent_cells(int x_map, int y_map, int dist);
    bool add_adjacent_cells(int x, int y, int dist);
    p WorldToMap(double x_w, double y_w);
    void add_walls_to_map(std::vector<i4tuple> Walls);
    void updateMinValues(double x1, double y1, double x2, double y2);
    void addCost(const int startX, const int startY, const int width, const int height, const int cost);
};
