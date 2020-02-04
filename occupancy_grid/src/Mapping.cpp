#include <ros/ros.h>
#include "Mapping.h"

visualization_msgs::Marker line_strip;
ros::Publisher marker_pub;

Mapping::Mapping() {}

/*----------------------------------------------------------------------------*/
// Callback functions
/*----------------------------------------------------------------------------*/

void Mapping::gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  // ROS_INFO("width: %d, height: %d", msg->info.width, msg->info.height);
  grid = *msg;
  map_width = msg->info.width;
  map_height = msg->info.height;
  resolution = msg->info.resolution;
}

void Mapping::scanCallback (const sensor_msgs::LaserScanConstPtr& msg) {
  angleIncrement = msg -> angle_increment;
  laser = msg -> ranges;
}

void Mapping::positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  position.x = msg -> x;
  position.y = msg -> y;
  position.z = msg -> z;
}

void Mapping::odometryCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  deltaX = msg -> x;
  deltaY = msg -> y;
  deltaTheta = msg -> z;
  ROS_INFO("deltaX: %f, deltaY: %f, deltaTheta: %f", msg -> x, msg -> y, msg -> z);
}

/*----------------------------------------------------------------------------*/
// Help functions
/*----------------------------------------------------------------------------*/
bool Mapping::areEqual(float x, float y) {
  return fabs(x - y) < 1e-6;
}

bool Mapping::smallerOrEqual(float x, float y) {
  return (x < y || fabs(x - y) < 1e-6);
}

bool Mapping::greaterOrEqual(float x, float y) {
  return (x > y || fabs(x - y) < 1e-6);
}

float Mapping::min(float x1, float x2) {
  if (x1 < x2) {
    return x1;
  }
  return x2;
}

float Mapping::max(float x1, float x2) {
  if (x1 > x2) {
    return x1;
  }
  return x2;
}

/*----------------------------------------------------------------------------*/
// Mapping
/*----------------------------------------------------------------------------*/
void Mapping::mapping(float xm, float ym, float thetam) {
  float theta; // ray angle
  float x, y; // intersection points
  float distance, minPositiveDistance, maxNegativeDistance;
  float cosTheta, sinTheta;
  float cost, sint; //transform from robot center to lidar
  int lLeft, lRight;
  float likelihood[360];
  point lidarPoints[360];

  for(lRight = 0; lRight < laser.size() / 2; lRight++) {
    lLeft = (lRight + 180);

    if(!std::isinf(laser[lLeft]) || !std::isinf(laser[lRight])) {
      theta = thetam + angleIncrement * lRight;

      cosTheta = cos(theta);
      sinTheta = sin(theta);
      cost = 0.075 * cos(thetam);
      sint = 0.075 * sin(thetam);
      minPositiveDistance = 8;
      maxNegativeDistance = -8;

      for(int w = 0; w < wallSet.size(); w++) {
        // vertical walls
        if(wallSet[w].x1 == wallSet[w].x2) {
          if(!areEqual(cosTheta, 0.0)) {
            distance = (wallSet[w].x1 - xm + cost) / cosTheta;
            x = wallSet[w].x1;
          }
          else {
            distance = 8;
          }
          y = ym - sint + distance * sinTheta;
        }
        // horizontal walls
        else if(wallSet[w].y1 == wallSet[w].y2) {
          if(!areEqual(sinTheta, 0.0)) {
            distance = (wallSet[w].y1 - ym + sint) / sinTheta;
            y = wallSet[w].y1;
          }
          else {
            distance = 8;
          }
          x = xm - cost + distance * cosTheta;
        }
        // other walls
        else {
          distance = (wallSet[w].m * (xm - cost) + wallSet[w].q - ym + sint) / (sinTheta - cosTheta * wallSet[w].m);
          x = xm - cost + distance * cosTheta;
          y = ym - sint + distance * sinTheta;
        }

        // check that intersection point is within the wall boundaries and find min / max
        if(greaterOrEqual(x, wallSet[w].x1) && smallerOrEqual(x, wallSet[w].x2) && greaterOrEqual(y, wallSet[w].y1) && smallerOrEqual(y, wallSet[w].y2)) {
          if(distance > 0) {
            if(distance < minPositiveDistance) {
              minPositiveDistance = distance;
            }
          }
          else if(distance > maxNegativeDistance) {
            maxNegativeDistance = distance;
          }
        }

      }

      if(!std::isinf(laser[lRight])) {

        likelihood[lRight] = fabs(laser[lRight] + maxNegativeDistance);
        lidarPoints[lRight].x = xm - cost - laser[lRight] * cosTheta;
        lidarPoints[lRight].y = ym - sint - laser[lRight] * sinTheta;
      }
      else {
        likelihood[lRight] = -1;
      }
      if(!std::isinf(laser[lLeft])) {
        likelihood[lLeft] = fabs(laser[lLeft] - minPositiveDistance);
        lidarPoints[lLeft].x = xm - cost + laser[lLeft] * cosTheta;
        lidarPoints[lLeft].y = ym - sint + laser[lLeft] * sinTheta;
      }
      else {
        likelihood[lLeft] = -1;
      }
    }
    else {
      likelihood[lRight] = -1;
      likelihood[lLeft] = -1;
    }
  }

  occupancy_grid::points_msg points_to_add;
  bool addPoints = false;
  bool closeToWall;
  float avgBefore, avgAfter;
  int nBefore, nAfter;
  int x_idx, y_idx;
  int idx;

  for (int i = 0; i < 360; i++) {
    avgAfter = avgBefore = 0.0;
    nBefore = nAfter = 0;

    // calculate windows averages
    for (int j = 0; j < WINDOW_SIZE; j++) {
      if(likelihood[(i+j+1) % 360] != -1) {
        avgAfter += log(likelihood[(i+j+1) % 360] + 1);
        nAfter++;
      }

      idx = i - j;
      if(idx < 0){
        idx = 360 + idx;
      }
      if(likelihood[idx % 360] != -1) {
        avgBefore += log(likelihood[idx % 360] + 1);
        nBefore++;
      }
    }
    avgAfter /= nAfter;
    avgBefore /= nBefore;

    // special case: start between rising edge and falling edge
    if (i == 0 && avgBefore > THRESHOLD_LIKELIHOOD && avgAfter > THRESHOLD_LIKELIHOOD){
      addPoints = true;
    }

    // rising edge
    if(avgBefore < THRESHOLD_LIKELIHOOD && avgAfter > THRESHOLD_LIKELIHOOD) {
      addPoints = true;
    }
    // falling edge
    else if(addPoints && avgBefore > THRESHOLD_LIKELIHOOD && avgAfter < THRESHOLD_LIKELIHOOD) {
      addPoints = false;
    }
    // ROS_INFO("IM HERE");
    if(addPoints) {
      if(likelihood[i] != -1) {
        if(laser[i] < THRESHOLD_DISTANCE){
          occupancy_grid::point_msg p;
          x_idx = (int)(lidarPoints[i].x / resolution);
          y_idx = (int)(lidarPoints[i].y / resolution);

          closeToWall = false;

          if(x_idx >= 0 && x_idx < map_height && y_idx >= 0 && y_idx < map_width) {
            if(grid.data[y_idx * map_width + x_idx] >= 75) {
              closeToWall = true;
            }
            // ROS_INFO("%d", frequency[x_idx][y_idx]);
          }
          else {
            closeToWall = true;
          }

          if(!closeToWall) {
            frequency[x_idx][y_idx]++;
            if(frequency[x_idx][y_idx] > MAX_FREQUENCY) {
              p.x = lidarPoints[i].x;
              p.y = lidarPoints[i].y;
              points_to_add.points.push_back(p);
            }
          }
        }
      }
    }
  }
  // Send points to grid map
  if(points_to_add.points.size() > 0) {
    pub_points.publish(points_to_add);
    // ROS_INFO("2");
  }
}

void Mapping::getWalls(std::string _map_file) {

  map_fs.open(_map_file.c_str());
  wall w;
  while (getline(map_fs, line)){

    if (line[0] == '#') {
        continue;
    }

    std::istringstream line_stream(line);
    line_stream >> x1 >> y1 >> x2 >> y2;

    w.x1 = x1;
    w.x2 = x2;
    w.y1 = y1;
    w.y2 = y2;

    if (x2 == x1) {
      w.m = 0;
      w.q = 0;
    }
    else if(y2 == y1) {
      w.m = 0;
      w.q = y1;
    }
    else {
      w.m = ((float)y2 - y1) / (x2 - x1);
      w.q =  y1 - w.m * x1;
    }

    w.x1 = min(x1, x2);
    w.x2 = max(x1, x2);
    w.y1 = min(y1, y2);
    w.y2 = max(y1, y2);

    wallSet.push_back(w);
  }
}

void Mapping::main(int argc, char** argv)
{
  ros::init(argc, argv, "mapping");
  ros::NodeHandle n;

  //resolution = 0.03; // TODO use parameter server

  line_strip.header.frame_id = "/base_map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;

  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.01;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  n.param<std::string>("map_file", _map_file, "maze_map.txt");
  getWalls(_map_file);
  // Publishers
  //pub_line = n.advertise<occupancy_grid::line_msg>("/Mapping/line", 10);
  pub_points = n.advertise<occupancy_grid::points_msg>("/Mapping/points", 10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // ROS_INFO("1");
  // Subscribers
  sub_grid = n.subscribe("/map", 1, &Mapping::gridCallback, this);
  sub_laser = n.subscribe("/scan", 1, &Mapping::scanCallback, this);
  sub_position = n.subscribe("/localisation/position", 1, &Mapping::positionCallback, this);
  sub_odom = n.subscribe("/localisation/odometry/increment", 1, &Mapping::odometryCallback, this);

  ros::Rate r(10);

  while(n.ok() && !laser.size() > 0) {
    ros::spinOnce();
  }
  while(n.ok() && !grid.data.size() > 0) {
    ros::spinOnce();
  }

  frequency = Mapping::matrix(map_height,std::vector<int>(map_width));
  // frequency = new int*[map_height];
  // for(int i = 0; i < map_height; ++i) {
  //   frequency[i] = new int[map_width];
  //   for(int j = 0; j < map_width; j++) {
  //     frequency[i][j] = 0;
  //   }
  // }

  while (ros::ok())
  {
    ros::spinOnce();
    // ROS_INFO("deltaTheta: %f, deltaX: %f, deltaY: %f", fabs(deltaTheta), fabs(deltaX), fabs(deltaY) );
    if (fabs(deltaTheta) < 5e-2 && fabs(deltaX) < 1e-1 && fabs(deltaY) < 1e-1) {
      mapping(position.x, position.y, position.z);
    }

    r.sleep();
  }
}

int main(int argc, char **argv)
{
  Mapping map_obj = Mapping();
  map_obj.main(argc, argv);
}
