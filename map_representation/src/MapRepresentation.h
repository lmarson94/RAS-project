#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tuple>

#include <cmath>

#include <iostream>
#include <fstream>

using namespace std;

class MapRepresentation
{
  typedef std::tuple<double, double, double, double> i4tuple;

  public:
    void main(int argc, char **argv);
    MapRepresentation();

  private:
    int count, wall_id;
    double x1, y1, x2, y2, x_max, y_max, x_least, y_least;
    string line, _map_file;

    visualization_msgs::Marker wall;
    visualization_msgs::MarkerArray walls;
    geometry_msgs::Point p;

    ros::Publisher marker_pub;
    ros::Publisher grid_size_pub;

    double calc_dist(double x1, double x2, double y1, double y2);
    double calc_angle(double x1, double x2, double y1, double y2);
    void addWalls(std::vector<i4tuple> Walls, std::vector<string> text_vec);
    void updateMinMaxValues(double x1, double y1, double x2, double y2);
};
