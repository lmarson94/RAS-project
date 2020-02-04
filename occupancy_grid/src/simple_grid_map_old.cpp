#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tuple>
#include <iostream>
#include <vector>
#include <math.h>


typedef std::tuple<int, int> i2tuple;
typedef std::vector<std::vector<int>> matrix;

std::vector<i2tuple> ray_trace(i2tuple start, i2tuple end)
{
  std::vector<i2tuple> traversed;

  int start_x = std::get<0>(start);
  int start_y = std::get<1>(start);

  int end_x = std::get<0>(end);
  int end_y = std::get<1>(end);

  int x = start_x;
  int y = start_y;

  std::tuple<float,float> dxdy = std::make_tuple(fabs(end_x - start_x),fabs(end_y - start_y));
  float dx = std::get<0>(dxdy);
  float dy = std::get<1>(dxdy);
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

void matinput(matrix& grid)
{
  ROS_INFO("TJENA");
  return;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "simple_grid_map");

  ros::NodeHandle n;

  geometry_msgs::Pose pose;
  geometry_msgs::Point point;
  geometry_msgs::Quaternion q;

  const int width = 4;
  const int height = 4;
  const float resolution = 0.1;

  const int n_width = (int)(width/resolution);
  const int n_height = (int)(height/resolution);
  const int n_elements = n_width*n_height;
  ROS_INFO("n_width = %d",n_width);
  ROS_INFO("n_height = %d",n_height);

  int map_origin_x = (int)(-resolution*n_width/2);
  int map_origin_y = (int)(-resolution*n_height/2);


  map_origin_x = 0;
  map_origin_y = 0;

  ROS_INFO("Map_origin_x = %d",map_origin_x);
  ROS_INFO("Map_origin_y = %d",map_origin_y);

  point.x = map_origin_x;
  point.y = map_origin_y;
  point.z = 0;

  q.x, q.y, q.z, q.w = 0;

  pose.position = point;
  pose.orientation = q;

  nav_msgs::OccupancyGrid map;
  map.info.resolution = resolution;      // float32
  map.info.width      = n_width;           // uint32
  map.info.height     = n_height;          // uint32
  map.info.origin     = pose;
  map.header.frame_id   = "odom";

  //std::vector<std::vector<int> > grid(n_height,std::vector<int>(n_width));
  matrix grid = matrix(n_height,std::vector<int>(n_width));
  matinput(grid);
  std::vector<signed char> p(n_elements);
  //int grid[n_height][n_width] = {};
  //int p[n_elements] = {};

  int x1 = 0;
  int y1 = 0;
  int x2 = 2;
  int y2 = 2;
  ROS_INFO("x1 = %d, y1 = %d", x1, y1);
  ROS_INFO("x2 = %d, y2 = %d", x2, y2);

  int map_idx_x1 = (int)((x1-map_origin_x)/resolution);
  int map_idx_y1 = (int)((y1-map_origin_y)/resolution);

  int map_idx_x2 = (int)((x2-map_origin_x)/resolution);
  int map_idx_y2 = (int)((y2-map_origin_y)/resolution);

  ROS_INFO("Map_idx_x1 = %d, Map_idx_y1 = %d", map_idx_x1, map_idx_y1);
  ROS_INFO("Map_idx_x2 = %d, Map_idx_y2 = %d", map_idx_x2, map_idx_y2);

  //grid[map_idx_x1][map_idx_y1] = 100;
  //grid[map_idx_x2][map_idx_y2] = 100;


  i2tuple start = i2tuple(map_idx_x1,map_idx_y1);
  i2tuple end = i2tuple(map_idx_x2,map_idx_y2);

  std::vector<i2tuple> ray = ray_trace(start,end);

  for(i2tuple t: ray)
  {
    int map_idx_x = std::get<0>(t);
    int map_idx_y = std::get<1>(t);

    //ROS_INFO("map_idx_x = %d, map_idx_y = %d",map_idx_x, map_idx_y);
    //ROS_INFO("map_idx_x = %d, map_idx_y = %d",map_idx_x, map_idx_y);
    if( (map_idx_x >= 0 && map_idx_x < n_width) && (map_idx_y >= 0 && map_idx_y < n_height))
    {
      //ROS_INFO("map_idx_x = %d, map_idx_y = %d",map_idx_x, map_idx_y);
      grid[map_idx_y][map_idx_x] = 100;
      //ROS_INFO("ray_x = %d, ray_y = %d",std::get<0>(t),std::get<1>(t));
      // ROS_INFO("map_x = %d, map_y = %d",map_idx_x, map_idx_y);
    }

  }

  int idx = 0;
  for (int i = 0; i < n_height; i++) {
    for (int j = 0; j < n_width; j++) {
      //ROS_INFO("grid[%d][%d] = %d",i,j,grid[i][j]);
      p[idx] = grid[i][j];
      idx++;
    }
  }

  //std::vector<signed char> a(p, p+n_elements);
  map.data = p;

  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 10);


  ros::Rate r(10);

  while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
  {
    map_pub.publish(map);

    ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

    r.sleep(); // Sleep for the rest of the cycle to enforce the loop rate
  }
  return 0;
}
