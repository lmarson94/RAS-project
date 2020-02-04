#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <random>
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

class ParticleFilter
{
  struct particle {
    float x;
    float y;
    float theta;
    float weight;
  };

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

  public:
    ParticleFilter();
    void run(int argc, char** argv);
    typedef std::tuple<int, int> i2tuple;

  private:
    int numberOfParticles;
    particle *particleSet, *resampledParticleSet;
    float *cdf;
    float x, y, theta;
    float smoothingFactor;
    float deltaX, deltaY, deltaTheta;
    float xNoise, yNoise, thetaNoise;
    float xOld, yOld, thetaOld;
    float xMean, yMean, sinThetaMean, cosThetaMean, thetaMean;
    float measurementInvVariance;
    float geqOne;
    float map_width, map_height;
    float weight_sum;
    std::default_random_engine generator;
    float resolution;

    float angleIncrement;
    std::vector<float> laser;

    std::vector<wall> wallSet;
    std::vector<signed char> grid;
    ros::Subscriber sub_grid, sub_laser, sub_odom;
    ros::Publisher pub_position;

    void printStatistics(float sum);
    float areEqual(float x, float y);
    float smallerOrEqual(float x, float y);
    float greaterOrEqual(float x, float y);
    float min(float x1, float x2);
    float max(float x1, float x2);
    float random(float max);
    float wrapPosition(float x, float max);


    void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScanConstPtr& cloud_msg);
    void odomCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    void initialization();
    void predictionAndUpdate();
    void updateNew();
    void resample();

    void ransac(std::vector<point> newWall);
    void mapping(float xm, float ym, float thetam);
    std::vector<i2tuple> get_line(i2tuple start, i2tuple end);


};
