#include <ros/ros.h>
#include "Mapping.h"

visualization_msgs::Marker line_strip;
ros::Publisher marker_pub;

Mapping::Mapping()
{

}

/*----------------------------------------------------------------------------*/
// Callback functions
/*----------------------------------------------------------------------------*/

void Mapping::gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  grid = *msg;
  map_width = msg->info.width;
  map_height = msg->info.height;
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
  float xmax, ymax, xmin, ymin;
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
              xmin = xm - cost + laser[lRight] * cosTheta;
              ymin = ym - sint + laser[lRight] * sinTheta;
            }
          }
          else if(distance > maxNegativeDistance) {
            maxNegativeDistance = distance;
            xmax = xm - cost - laser[lRight] * cosTheta;
            ymax = ym - sint - laser[lRight] * sinTheta;
          }
        }

      }

      if(!std::isinf(laser[lRight])) {

        likelihood[lRight] = fabs(laser[lRight] + maxNegativeDistance);
        lidarPoints[lRight].x = xmax;
        lidarPoints[lRight].y = ymax;
      }
      else {
        likelihood[lRight] = -1;
      }
      if(!std::isinf(laser[lLeft])) {
        likelihood[lLeft] = fabs(laser[lLeft] - minPositiveDistance);
        lidarPoints[lLeft].x = xmin;
        lidarPoints[lLeft].y = ymin;
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

  float threshold = log(1 + 0.4);
  float avgBefore, avgAfter;
  int windowSize = 5;
  bool addPoints;
  std::vector<std::vector<point>> walls;
  std::vector<point> newWall;

  for (int i = 0; i < 360; i++) {
    avgAfter = avgBefore = 0.0;

    for (int j = i; j < windowSize; j++) {
      if(likelihood[j+1 % 360] != -1)
        avgAfter += log(likelihood[j+1 % 360] + 1);

      if(likelihood[360 - j % 360] != -1)
        avgBefore += log(likelihood[360 - j % 360] + 1);
    }
    avgAfter /= windowSize;
    avgBefore /= windowSize;

    if (i == 0 && avgBefore > threshold && avgAfter > threshold){
      addPoints = true;
    }

    if (i == 359 && avgBefore > threshold && avgAfter > threshold){
      walls[0].insert(walls[0].end(), newWall.begin(), newWall.end());
    }

    // rising edge
    if(avgBefore < threshold && avgAfter > threshold) {
      addPoints = true;
    }
    // falling edge
    else if(avgBefore > threshold && avgAfter < threshold) {
      addPoints = false;
      walls.push_back(newWall);
      newWall.clear();
    }

    if(addPoints) {
      newWall.push_back(lidarPoints[i]);
    }
  }
  // Fit lines with ransac
  ROS_INFO("walls size %lu", walls.size());
  for (int i = 0; i < walls.size(); i++) {
    ROS_INFO("%lu", walls[i].size());
    sequentialRansac(walls[i]);
  }
}

void Mapping::sequentialRansac(std::vector<point> newWall){
  wall w;
  point pOrthogonal;
  std::vector<point> tmp;
  float thresholdDistance = 0.2;
  float xmin = 999999.0;
  float xmax = 0.0;
  float distance;

  while(newWall.size() > 0) { // DEBUG increase minimum size
      w = ransac(newWall, thresholdDistance);

      for(int i = 0; i < newWall.size(); i++) {
        pOrthogonal.x = (newWall[i].y + w.m * newWall[i].x - w.q) / (2 * w.m);
        pOrthogonal.y = w.m * pOrthogonal.x + w.q;

        distance = sqrt(pow(pOrthogonal.x - newWall[i].x, 2) + pow(pOrthogonal.y - newWall[i].y, 2));

        if(distance < thresholdDistance) {
          if(newWall[i].x < xmin)
            xmin = newWall[i].x;
          else if(newWall[i].x > xmax)
            xmax = newWall[i].x;
        }
        else if(distance >= thresholdDistance) {
          tmp.push_back(newWall[i]);
        }
      }

      w.x1 = xmin;
      w.y1 = min(xmin * w.m + w.q, xmax * w.m + w.q);
      w.x2 = xmax;
      w.y2 = min(xmax * w.m + w.q, xmax * w.m + w.q);
      wallSet.push_back(w);

      geometry_msgs::Point p;
      line_strip.points.clear();
      p.x = w.x1;
      p.y = xmin * w.m + w.q;
      p.z = 0.0;
      line_strip.points.push_back(p);
      p.x = w.x2;
      p.y = xmax * w.m + w.q;
      p.z = 0.0;
      line_strip.points.push_back(p);
      marker_pub.publish(line_strip);
      line_strip.id++;

      newWall = tmp;
      tmp.clear();
  }
}

Mapping::wall Mapping::ransac(std::vector<point> newWall, float thresholdDistance) {
  int maxVote = 0;
  int iteration = 0;
  int lastUpdate = 0;
  int r1, r2;
  int vote;
  float mStar, qStar, m, q;
  float distance;
  point p1, p2, pOrthogonal;
  wall w;

  int thresholdIteration = (int) (0.1 * newWall.size());

  bool converged = false;
  while(!converged) {
    r1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (newWall.size())));
    r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (newWall.size())));
    while(r1 == r2) {
      r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (newWall.size())));
    }

    p1.x = newWall[r1].x;
    p1.y = newWall[r1].y;
    p2.x = newWall[r2].x;
    p2.y = newWall[r2].y;

    m = (p1.y - p2.y) / (p1.x - p2.x);
    q = p1.y - m * p1.x;

    vote = 0;
    for(int i = 0; i < newWall.size(); i++) {
      if(i == r1 || i == r2)
        continue;

      pOrthogonal.x = (newWall[i].y + m * newWall[i].x - q) / (2 * m);
      pOrthogonal.y = m * pOrthogonal.x + q;

      distance = sqrt(pow(pOrthogonal.x - newWall[i].x, 2) + pow(pOrthogonal.y - newWall[i].y, 2));
      if(distance < thresholdDistance) {
        vote++;
      }
    }

    if(vote > maxVote) {
      maxVote = vote;
      mStar = m;
      qStar = q;
      lastUpdate = iteration;
    }

    if(iteration - lastUpdate > thresholdIteration) {
      converged = true;
    }

    iteration++;
  }

  w.m = mStar;
  w.q = qStar;
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
  // Publishers
  //pub_grid = n.advertise<nav_msgs::OccupancyGrid>("/map", 10);
  //pub_lines = n.advertise<nav_msgs::OccupancyGrid>("/wall_lines", 10);

  // Subscribers
  sub_grid = n.subscribe("/map", 1, &Mapping::gridCallback, this);
  sub_laser = n.subscribe("/scan", 1, &Mapping::scanCallback, this);
  sub_position = n.subscribe("/localisation/position", 1, &Mapping::positionCallback, this);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    mapping(position.x, position.y, position.z);
    //pub_grid.publish(grid);

    r.sleep();
  }
  return;
}

int main(int argc, char **argv)
{
  Mapping map_obj = Mapping();
  map_obj.main(argc, argv);
}
