#include "ParticleFilter.h"
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
visualization_msgs::Marker line_strip;
typedef std::tuple<int, int> i2tuple;
ros::Publisher map_pub;
nav_msgs::OccupancyGrid map;
bool readMap = false;

ParticleFilter::ParticleFilter() {
  // particle filter parameters

  numberOfParticles = 800;
  particleSet = new particle[numberOfParticles];
  resampledParticleSet = new particle[numberOfParticles];
  cdf = new float[numberOfParticles];

  xOld = x;
  yOld = y;
  thetaOld = theta;

  xNoise = 0.014;
  yNoise = 0.014;
  thetaNoise = 0.07;
  measurementInvVariance = 10000;

  geqOne = nextafterf(1.0, 2.0);

  smoothingFactor = 0.35;
}

/*----------------------------------------------------------------------------*/
// Callback functions
/*----------------------------------------------------------------------------*/

void ParticleFilter::gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  if(readMap == false) {
    map = *msg;
    map_width = msg->info.width;
    map_height = msg->info.height;
    grid = msg->data;
    resolution = msg->info.resolution;
  }
}

void ParticleFilter::scanCallback (const sensor_msgs::LaserScanConstPtr& msg) {
  angleIncrement = msg -> angle_increment;
  laser = msg -> ranges;
}

void ParticleFilter::odomCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  deltaX = msg -> x;
  deltaY = msg -> y;
  deltaTheta = msg -> z;
}

/*----------------------------------------------------------------------------*/
// Utility functions
/*----------------------------------------------------------------------------*/

void ParticleFilter::printStatistics(float sum) {
  float mean = sum / numberOfParticles;
  float sumVar = 0;

  for(int i = 0; i < numberOfParticles; i++) {
    sumVar += pow(particleSet[i].weight/weight_sum - mean,2);
  }

  ROS_INFO("mean: %f variance: %f", mean, sumVar / numberOfParticles);
}

float ParticleFilter::areEqual(float x, float y) {
  return fabs(x - y) < 1e-6;
}

float ParticleFilter::smallerOrEqual(float x, float y) {
  return (x < y || fabs(x - y) < 1e-6);
}

float ParticleFilter::greaterOrEqual(float x, float y) {
  return (x > y || fabs(x - y) < 1e-6);
}

float ParticleFilter::min(float x1, float x2) {
  if (x1 < x2) {
    return x1;
  }
  return x2;
}

float ParticleFilter::max(float x1, float x2) {
  if (x1 > x2) {
    return x1;
  }
  return x2;
}

float ParticleFilter::random(float stddev) {
  std::normal_distribution<double> dist(0, stddev);
  return dist(generator);
}

float ParticleFilter::wrapPosition(float x, float max) {
  if(x < 0.0) {
    x = 0.0;
  }
  else if(x > max) { // TODO set maximum value
    x = max;
  }

  return x;
}

/*----------------------------------------------------------------------------*/
// Particle filter functions
/*----------------------------------------------------------------------------*/

void ParticleFilter::initialization() {
  for(int i = 0; i < numberOfParticles; i++) {
    particleSet[i].x = wrapPosition(x + random(0.01), map_width * resolution);
    particleSet[i].y = wrapPosition(y + random(0.01), map_height * resolution);
    particleSet[i].theta = theta + random(0.009);
    particleSet[i].weight = 0.0;
  }
}

void ParticleFilter::predictionAndUpdate() {
  float theta; // ray angle
  float x, y; // intersection points
  int x_idx, y_idx; // grid coordinates
  float distance, minPositiveDistance, maxNegativeDistance;
  float cosTheta, sinTheta;
  float cost, sint; //transform from robot center to lidar
  int lLeft, lRight;
  float xmax, ymax, xmin, ymin;

  weight_sum = 0.0; // weights normalization constant
  //ROS_INFO("walls %d", wallSet.size());
  for(int i = 0; i < numberOfParticles; i++) {
    particleSet[i].x = wrapPosition(particleSet[i].x + deltaX + random(xNoise), map_width * resolution);
    particleSet[i].y = wrapPosition(particleSet[i].y + deltaY + random(yNoise), map_height * resolution);
    particleSet[i].theta = particleSet[i].theta + deltaTheta + random(thetaNoise);

    x_idx = static_cast<int>(particleSet[i].x * 300); // TODO set resolution
    y_idx = static_cast<int>(particleSet[i].y * 300);

    if(grid[y_idx * map_width + x_idx] == 100) {
      particleSet[i].weight = 0.0;
    }
    else {
      for(lRight = 0; lRight < laser.size() / 2; lRight += 2) {
        lLeft = (lRight + 180);

        if(!std::isinf(laser[lLeft]) || !std::isinf(laser[lRight])) {
          theta = particleSet[i].theta + angleIncrement * lRight;

          cosTheta = cos(theta);
          sinTheta = sin(theta);
          cost = 0.075 * cos(particleSet[i].theta);
          sint = 0.075 * sin(particleSet[i].theta);
          minPositiveDistance = 8;
          maxNegativeDistance = -8;

          for(int w = 0; w < wallSet.size(); w++) {
            // vertical walls
            if(wallSet[w].x1 == wallSet[w].x2) {
              if(!areEqual(cosTheta, 0.0)) {
                distance = (wallSet[w].x1 - particleSet[i].x + cost) / cosTheta;
                x = wallSet[w].x1;
              }
              else {
                distance = 8;
              }
              y = particleSet[i].y - sint + distance * sinTheta;
            }
            // horizontal walls
            else if(wallSet[w].y1 == wallSet[w].y2) {
              if(!areEqual(sinTheta, 0.0)) {
                distance = (wallSet[w].y1 - particleSet[i].y + sint) / sinTheta;
                y = wallSet[w].y1;
              }
              else {
                distance = 8;
              }
              x = particleSet[i].x - cost + distance * cosTheta;
            }
            // other walls
            else {
              distance = (wallSet[w].m * (particleSet[i].x - cost) + wallSet[w].q - particleSet[i].y + sint) / (sinTheta - cosTheta * wallSet[w].m);
              x = particleSet[i].x - cost + distance * cosTheta;
              y = particleSet[i].y - sint + distance * sinTheta;
            }

            // check that intersection point is within the wall boundaries and find min / max
            if(greaterOrEqual(x, wallSet[w].x1) && smallerOrEqual(x, wallSet[w].x2) && greaterOrEqual(y, wallSet[w].y1) && smallerOrEqual(y, wallSet[w].y2)) {
              if(distance > 0) {
                if(distance < minPositiveDistance) {
                  minPositiveDistance = distance;
                  xmin = x;
                  ymin = y;
                }
              }
              else if(distance > maxNegativeDistance) {
                maxNegativeDistance = distance;
                xmax = x;
                ymax = y;
              }
            }

          }

          // assign weights
          if(!std::isinf(laser[lRight])) {
            particleSet[i].weight += exp(-(pow(laser[lRight] + maxNegativeDistance, 2)) * 0.5 * measurementInvVariance);
          }
          if(!std::isinf(laser[lLeft])) {
            particleSet[i].weight += exp(-(pow(laser[lLeft] - minPositiveDistance, 2)) * 0.5 * measurementInvVariance);
          }
        }
      }
    }
    weight_sum += particleSet[i].weight;
  }
}

void ParticleFilter::resample() {
  float sum = 0;
  float r0;

  for(int i = 0; i < numberOfParticles; i++) {
    sum += particleSet[i].weight / weight_sum;
    cdf[i] = sum;
  }
  cdf[numberOfParticles - 1] = geqOne;

  //printStatistics(sum);

  r0 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (1.0/numberOfParticles)));
  int j = 0;
  int count = 0;
  for(int i = 0; i < numberOfParticles; i++) {
    for(; j < numberOfParticles; j++) {
      if (cdf[j] >= r0 + (float)i / numberOfParticles) {
        break;
      }
      count++;
    }

    resampledParticleSet[i] = particleSet[j];
    resampledParticleSet[i].weight = 0.0;

    xMean += resampledParticleSet[i].x;
    yMean += resampledParticleSet[i].y;
    thetaMean += resampledParticleSet[i].theta;
  }
  particleSet = resampledParticleSet;
}

void ParticleFilter::run(int argc, char** argv) {
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle n;

  n.getParam("/x", x);
  n.getParam("/y", y);
  n.getParam("/theta", theta);

  // read map file and calculate walls parameters
  std::ifstream map_fs;
  std::string line, _map_file;

  n.param<std::string>("map_file", _map_file, "maze_map.txt");
  map_fs.open(_map_file.c_str());

  float x1, x2, y1, y2;
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

  pub_position = n.advertise<geometry_msgs::Vector3>("/localisation/position", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  sub_grid = n.subscribe("/map", 1, &ParticleFilter::gridCallback, this);
  sub_laser = n.subscribe("/scan", 1, &ParticleFilter::scanCallback, this);
  sub_odom = n.subscribe("/localisation/odometry/increment", 1, &ParticleFilter::odomCallback, this);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_updated", 10);

  geometry_msgs::Vector3 msg;

  ros::Rate loop_rate(10);

  // read grid and first laser scan
  while(n.ok() && !(grid.size() > 0 && laser.size() > 0)) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  readMap = true;

  // initialize particle set
  initialization();

  int x_idx, y_idx;
  while(n.ok()) {
    ros::spinOnce();

    xMean = 0;
    yMean = 0;
    thetaMean = 0;

    ROS_INFO("x %f y %f theta %f", deltaX, deltaY, deltaTheta);

    predictionAndUpdate();
    resample();

    // publish current estimated position
    msg.x = smoothingFactor * xMean / numberOfParticles + (1 - smoothingFactor) * (xOld + deltaX);
    msg.y = smoothingFactor * yMean / numberOfParticles + (1 - smoothingFactor) * (yOld + deltaY);
    msg.z = smoothingFactor * thetaMean / numberOfParticles + (1 - smoothingFactor) * (thetaOld + deltaTheta);
    pub_position.publish(msg);

    xOld = msg.x;
    yOld = msg.y;
    thetaOld = msg.z;

    loop_rate.sleep();
  }

}

int main(int argc, char **argv)
{
  ParticleFilter pf = ParticleFilter();
  pf.run(argc, argv);
}
