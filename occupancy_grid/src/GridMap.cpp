 #include <ros/ros.h>
#include "GridMap.h"
#include <tf/tf.h>
#include <cmath>
#include <iostream> // TODO: Remove
#include "OccupancyRegion.cpp"
#include <fstream>
#include <sstream>
#include <string>


double width;
double height;
double x_new, y_new, orientation_new;
bool new_obstacle = false;
bool new_points = false;
bool new_object = false;
occupancy_grid::points_msg point_vec;
geometry_msgs::Point obj;
const int R1_obs = 4;                    // Inflation radius 1
const int R2_obs = 6;                    // Inflation radius 2
const int R3_obs = 3;                    // Inflation radius 3
const int R_obj = 3;

bool newMap = false;


GridMap::GridMap(int n_width, int n_height, double resolution, string _map_file = "lab_maze_2018.txt")
{
  frame_id = "base_map";

  // Position of map
  point.x = map_origin_x;
  point.y = map_origin_y;
  point.z = 0;

  // Orientation of map
  tf::Quaternion q; q.setRPY(0.0,0.0,0.0); // (roll(around x), pitch(around y), yaw(around z))
  tf::quaternionTFToMsg(q, pose.orientation);
  //q.x, q.y, q.z, q.w = 0;

  // Set Pose
  pose.position = point;
  //pose.orientation = q;

  // Set data for OccupancyGrid msg
  map.info.resolution = resolution;
  map.info.origin     = pose;
  map.header.frame_id = frame_id;
  map.header.stamp = ros::Time();

  n_elements = n_width*n_height;
  map.info.width      = n_width;
  map.info.height     = n_height;
  vec = std::vector<signed char>(n_elements);
  map.data        = vec;
  this->n_width = n_width;
  this->n_height = n_height;
  this->resolution = resolution;

  new_obstacle = false;

  x_least = y_least = 9999.0;

  int n_elements = n_width*n_height;

  ifstream map_fs; map_fs.open(_map_file.c_str());
  std::vector<i4tuple> walls;
  while (getline(map_fs, line)){

      if (line[0] == '#') {
          continue;
      }

    std::istringstream line_stream(line);
    line_stream >> x1 >> y1 >> x2 >> y2;
    walls.push_back(GridMap::i4tuple(x1,y1,x2,y2));
    updateMinValues(x1, y1, x2, y2);
  }
  add_walls_to_map(walls);

  // Mark cells outside the maze as explored
  if (map.data[n_width*n_width+n_height] != 100){
   for (int col = 0; col < n_width; col++) {
     for (int row = 0; row < n_height; row++) {
       if (map.data[row*n_width+col] == 0) map.data[row*n_width+col] = -2;
       else break;
     }
     for (int row = n_height-1; row >= 0; row--) {
       if (map.data[row*n_width+col] == 0) map.data[row*n_width+col] = -2;
       else break;
     }
   }
  }

}

bool GridMap::inBounds(int x, int y)
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

std::vector<GridMap::i2tuple> GridMap::get_line(GridMap::i2tuple start, GridMap::i2tuple end)
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

void GridMap::updateMap(GridMap::i2tuple start, GridMap::i2tuple end)
{
  std::vector<i2tuple> _line = get_line(start,end);

  for(i2tuple t: _line)
  {
    int map_idx_x = std::get<0>(t);
    int map_idx_y = std::get<1>(t);
    if (inBounds(map_idx_x, map_idx_y) == true)
    {
      map.data[map_idx_y*n_width+map_idx_x] = 100; // Occupied
    }
    else
    {
      ROS_INFO("Index out of bounds: (%d,%d)",map_idx_x, map_idx_y);
    }
  }
}

GridMap::p GridMap::WorldToMap(double x_w, double y_w)
{
  double Nx = (int)(width/resolution);
  double Ny = (int)(height/resolution);
  double x,y;
  p map_idx;

  x = x_w - map_origin_x;
  y = y_w - map_origin_y;

  // Grid Coordinates
  x /= resolution;
  y /= resolution;

  x -= x_least/resolution;
  y -= y_least/resolution;

  map_idx.x = x;
  map_idx.y = y;

  return map_idx;

}

void GridMap::updateMinValues(double x1, double y1, double x2, double y2)
{
  if (x1 < x_least) x_least = x1;

  if (y1 < y_least) y_least = y1;

  if (x2 < x_least) x_least = x2;

  if (y2 < y_least) y_least = y2;
}

void GridMap::add_walls_to_map(std::vector<i4tuple> Walls)
{

  for(i4tuple t: Walls)
  {
    double x1 = std::get<0>(t);
    double y1 = std::get<1>(t);
    double x2 = std::get<2>(t);
    double y2 = std::get<3>(t);

    p map_idx_1, map_idx_2;
    map_idx_1 = WorldToMap(x1,y1);
    map_idx_2 = WorldToMap(x2,y2);

    i2tuple start = i2tuple(map_idx_1.x,map_idx_1.y);
    i2tuple end = i2tuple(map_idx_2.x,map_idx_2.y);

    updateMap(start, end);
  }

}

void GridMap::add_object_to_map(const geometry_msgs::Point msg)
{
  int map_idx_x = (int)((msg.x-map_origin_x)/resolution);
  int map_idx_y = (int)((msg.y-map_origin_y)/resolution);
  if (inBounds(map_idx_x,map_idx_y) && map.data[map_idx_y*n_width+map_idx_x] < 100) {
      map.data[map_idx_y*n_width+map_idx_x] = 100;
      inflateObstacle(R_obj, 25, map_idx_x, map_idx_y);
  }
}

void GridMap::add_point_to_map(const occupancy_grid::point_msg msg)
{
  int map_idx_x = (int)((msg.x-map_origin_x)/resolution);
  int map_idx_y = (int)((msg.y-map_origin_y)/resolution);
  if (inBounds(map_idx_x,map_idx_y) && map.data[map_idx_y*n_width+map_idx_x] < 75) {
      map.data[map_idx_y*n_width+map_idx_x] = 100;
      inflateObstacle(R2_obs, 25, map_idx_x, map_idx_y);
      inflateObstacle(R1_obs, 50, map_idx_x, map_idx_y);
      inflateObstacle(R3_obs, 75, map_idx_x, map_idx_y);
  }
}

void GridMap::inflateMap(const int R, const int cost)
{
  for (int i = 0; i < n_height; i++) {
    for (int j = 0; j < n_width; j++) {
      if (map.data[i*n_width+j] == 100)
      {
        int Y = R;
        for (int y = -Y; y < Y+1; y++) {
          int X = (int)(sqrt(pow(R,2) - pow(y,2)));
          for (int x = -X; x < X+1; x++) {
            if (map.data[(i+y)*n_width+(j+x)] != 100 && inBounds(j+x, i+y) == true)
            {
              map.data[(i+y)*n_width+(j+x)] = cost;
            }
          }
        }
      }
    }
  }
}
void GridMap::inflateObstacle(const int R, const int cost, int x_obs, int y_obs)
{
  if (map.data[y_obs*n_width+x_obs] == 100)
  {
    int Y = R;
    for (int y = -Y; y < Y+1; y++) {
      int X = (int)(sqrt(pow(R,2) - pow(y,2)));
      for (int x = -X; x < X+1; x++) {
        if (map.data[(y_obs+y)*n_width+(x_obs+x)] < 75 && inBounds(x_obs+x, y_obs+y) == true)
        {
          if (cost > map.data[(y_obs+y)*n_width+(x_obs+x)]) map.data[(y_obs+y)*n_width+(x_obs+x)] = cost;
        }
      }
    }
  }
}

void GridMap::addCost(const int startX, const int startY, const int width, const int height, const int cost)
{
    for (int i=0; i < height; i++)
    {
      for (int j=0; j < width; j++)
      {
        if (map.data[ (startY + i)*n_width + (j+startX)] < cost)
        {
          map.data[ (startY + i)*n_width + (j+startX)] = cost;
        }
      }
    }
}

nav_msgs::OccupancyGrid GridMap::getMap()
{
  return map;
}

void GridMap::adjacent_cells(int x_map, int y_map, int dist)
{
  std::vector<GridMap::i2tuple> adj_cells;
  int distance = 2;
  bool add = true;
  for (int i = 0; i < dist; i++) {
    for (int j = 0; j < dist; j++) {
      if (map.data[(y_map+i)*n_width+(x_map+j)] != 100 && inBounds(x_map+j, y_map+i) == true)
      {
        if (add_adjacent_cells(x_map+j, y_map+i, distance)){
          adj_cells.push_back(i2tuple(x_map+j,y_map+i));
        }
        else
        {
          add = false;
          return;
        }
      }
      if (map.data[(y_map+i)*n_width+(x_map-j)] != 100 && inBounds(x_map-j, y_map+i) == true)
      {
        if (add_adjacent_cells(x_map-j, y_map+i, distance)){
          adj_cells.push_back(i2tuple(x_map-j,y_map+i));
        }
        else
        {
          add = false;
          return;
        }
      }

      if (map.data[(y_map-i)*n_width+(x_map+j)] != 100 && inBounds(x_map+j, y_map-i) == true)
      {
        if (add_adjacent_cells(x_map+j, y_map-i, distance)){
          adj_cells.push_back(i2tuple(x_map+j,y_map-i));
        }
        else
        {
          add = false;
          return;
        }
      }

      if (map.data[(y_map-i)*n_width+(x_map-j)] != 100 && inBounds(x_map-j, y_map-i) == true)
      {
        if (add_adjacent_cells(x_map-j, y_map-i, distance)){
          adj_cells.push_back(i2tuple(x_map-j,y_map-i));
        }
        else
        {
          add = false;
          return;
        }
      }
    }
  }
  if (add)
  {
    for(i2tuple t: adj_cells)
    {
      int map_idx_x = std::get<0>(t);
      int map_idx_y = std::get<1>(t);
      map.data[map_idx_y*n_width+map_idx_x] = 100;

    }
  }
}

bool GridMap::add_adjacent_cells(int x, int y, int dist)
{
  for (int i = 0; i < dist; i++) {
    for (int j = 0; j < dist; j++) {
      if (map.data[(y+i)*n_width+(x+j)] == 100 || inBounds(x+j, y+i) == false) return false;

      if (map.data[(y+i)*n_width+(x-j)] == 100 || inBounds(x-j, y+i) == false) return false;

      if (map.data[(y-i)*n_width+(x+j)] == 100 || inBounds(x+j, y-i) == false) return false;

      if (map.data[(y-i)*n_width+(x-j)] == 100 || inBounds(x-j, y-i) == false) return false;
    }
  }
  return true;
}

void GridMap::add_obstacle_to_map(double x, double y, double orientation_new)
{
  if(orientation_new != 0.0)
  {
    int map_idx_x = (int)((x-map_origin_x)/resolution);
    int map_idx_y = (int)((y-map_origin_y)/resolution);

    if(orientation_new == 1.0){ // Vertical battery
      adjacent_cells(map_idx_x, map_idx_y, vertical);
      inflateObstacle(R2_obs+vertical, 25, map_idx_x, map_idx_y);
      inflateObstacle(R1_obs+vertical, 50, map_idx_x, map_idx_y);
      inflateObstacle(R3_obs+vertical, 75, map_idx_x, map_idx_y);
    }
    else if(orientation_new == 2.0){ // Horisonal battery
      adjacent_cells(map_idx_x, map_idx_y, horisontal);
      inflateObstacle(R2_obs+horisontal, 25, map_idx_x, map_idx_y);
      inflateObstacle(R1_obs+horisontal, 50, map_idx_x, map_idx_y);
      inflateObstacle(R3_obs+horisontal, 75, map_idx_x, map_idx_y);
    }
  }
}

void GridMap::insert_points_to_map(const occupancy_grid::points_msg msg)
{
  occupancy_grid::point_msg p;
  for (int i = 0; i < msg.points.size(); i++) {
    p.x = msg.points[i].x;
    p.y = msg.points[i].y;
    add_point_to_map(p);
  }
}

void grid_size_callback(const geometry_msgs::Point::ConstPtr& msg)
{
  width = msg->x;
  height = msg->y;
  // ROS_INFO("width: %f, height: %f", width, height);
}

void obstacle_callback(const geometry_msgs::Point::ConstPtr& msg)
{
  x_new = msg->x;
  y_new = msg->y;
  orientation_new = msg->z; // To indicate if battery is standing vertically or horisontally
  ROS_INFO("x_new: %f, y_new %f, ori_new: %f", x_new, y_new, orientation_new);
  new_obstacle = true;
}

void pointsCallback(const occupancy_grid::points_msg::ConstPtr& msg) {
  point_vec = *msg;
  new_points = true;
}

void objectCallback(const ras_object_master::ObjectMasterMsg::ConstPtr& msg) {
  obj.x = msg->point.point.x;
  obj.y = msg->point.point.y;
  obj.z = 0.0;
  ROS_INFO("x: %f, y: %f",msg->point.point.x,msg->point.point.y);
  new_object = true;
}

int main(int argc, char *argv[])
{
  const float resolution = 0.03;      // Resolution [m]
  const int R1 = 5;                    // Inflation radius 1
  const int R2 = 7;                    // Inflation radius 2
  const int R3 = 3;
  // const int R1_obs = 3;                    // Inflation radius 1
  // const int R2_obs = 5;                    // Inflation radius 2
  ros::init(argc, argv, "occupancy_grid");
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 10);
  ros::Publisher map_bounds_pub = n.advertise<geometry_msgs::Point>("/map_bounds", 10);
  ros::Subscriber grid_size_sub = n.subscribe("/grid_size", 1, grid_size_callback);
  ros::Subscriber obstacle_sub = n.subscribe("Geometry/New_Point", 1, obstacle_callback);
  ros::Subscriber points_sub = n.subscribe("/Mapping/points", 10, pointsCallback);
  ros::Subscriber object_sub = n.subscribe("/target/Geometry/New_Point", 1, objectCallback);

  ros::Duration(3).sleep();
  ros::spinOnce();

  // Map properties
  int n_width    = (int) ceil(width/resolution);
  int n_height   = (int) ceil(height/resolution);

  geometry_msgs::Point map_bounds_max;

  map_bounds_max.x = (float) n_width;
  map_bounds_max.y = (float) n_height;
  map_bounds_pub.publish(map_bounds_max);

  string _map_file;
  n.param<string>("map_file", _map_file, "maze_map.txt");
  GridMap grid = GridMap(n_width, n_height, resolution, _map_file);
  ROS_INFO("%s", argv[1]);
  if (std::string(argv[1]) == "1"){
    // TODO: READMAP
    std::vector<signed char> vec;

    ifstream inFile;

    inFile.open("/home/ras21/catkin_ws/src/ras_project_group1/occupancy_grid/src/occupancy_grid.txt");
    if (!inFile) {
        ROS_INFO("Unable to open file");
        exit(1); // terminate with error
    }
    int a;
    while (inFile >> a) {
        vec.push_back((signed char)a);
    }

    inFile.close();

    grid.map.data = vec; // data from file
  }
  else {
    grid.inflateMap(R2, 25);
    grid.inflateMap(R1, 50);
    grid.inflateMap(R3, 75);
  }
  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if(new_obstacle){
      //grid.add_point_to_map(x_new,y_new);
      if (orientation_new != 0.0){
        grid.add_obstacle_to_map(x_new,y_new,orientation_new);
      }
      new_obstacle = false;
    }
    if(new_points){
      grid.insert_points_to_map(point_vec);
    }
    new_points = false;

    if(new_object){
      grid.add_object_to_map(obj);
    }
    new_object = false;

    map_pub.publish(grid.getMap());
    r.sleep();
  }
}
