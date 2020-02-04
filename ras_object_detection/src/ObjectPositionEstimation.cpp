#include "ObjectPositionEstimation.h"

ObjectPositionEstimation::ObjectPositionEstimation() {

}

bool ObjectPositionEstimation::estimatePosition(ras_object_detection::EstimateObjectPosition::Request  &req,
                                                ras_object_detection::EstimateObjectPosition::Response &res)
{
  ras_object_detection::PointStampedArray msg;
  geometry_msgs::PointStamped pointMap;
  boundingBox b;

  int i;
  for(i = 0; i < req.objectList.coords.size(); i += 7) {
    b.xul = req.objectList.coords[i];
    b.yul = req.objectList.coords[i+1];
    b.xdr = req.objectList.coords[i+2];
    b.ydr = req.objectList.coords[i+3];

    try {
      pointMap = ObjectPositionEstimation::getAverageDepth(req.inputCloud, b);
      msg.points.push_back(pointMap);
    }
    catch( const char* msg ) {
      ROS_INFO("%s", msg);
    }
  }
  if(i > 0) {
    //ROS_INFO("Detected %d objects!!", msg.points.size());
    //pub_position.publish(msg)
    res.positions = msg;
    return true;
  }
  else{
    //res.positions = ???
  }
  return false;
}

geometry_msgs::PointStamped ObjectPositionEstimation::getAverageDepth(const sensor_msgs::PointCloud2 cloud, boundingBox b) {
  geometry_msgs::PointStamped pCamera, pMap;
  float x, y, z;
  float xAvg = 0.0;
  float yAvg = 0.0;
  float zAvg = 0.0;
  int xOff = cloud.fields[0].offset;
  int yOff = cloud.fields[1].offset;
  int zOff = cloud.fields[2].offset;
  int counter = 0;

  int dec_x = (b.xdr - b.xul) * 0.15;
  int dec_y = (b.ydr - b.yul) * 0.15;

  for (int i = b.xul+dec_x; i < b.xdr-dec_x; i++) {
    for (int j = b.yul+dec_y; j < b.ydr-dec_y; j++) {
      memcpy(&x, &(cloud.data[((cloud.width) * j + i) * cloud.point_step + xOff]), sizeof(float));
      memcpy(&y, &(cloud.data[((cloud.width) * j + i) * cloud.point_step + yOff]), sizeof(float));
      memcpy(&z, &(cloud.data[((cloud.width) * j + i) * cloud.point_step + zOff]), sizeof(float));

      if(!isnan(x) && !isnan(y) && !isnan(z)) {
        xAvg += x;
        yAvg += y;
        zAvg += z;
        counter++;
      }
    }
  }

  if (counter < 1)
  {
	pCamera.header.frame_id = "--";
  	pCamera.header.stamp = cloud.header.stamp;
  	pCamera.point.x = -99;
  	pCamera.point.y = -99;
  	pCamera.point.z = -99;
	return pCamera;
  }

  pCamera.header.frame_id = "camera";
  pCamera.header.stamp = cloud.header.stamp;
  pCamera.point.x = xAvg / counter;
  pCamera.point.y = yAvg / counter;
  pCamera.point.z = zAvg / counter;

  pListener -> transformPoint("base_map", pCamera, pMap);
  return pMap;

  //return pCamera; //delete later.
}

void ObjectPositionEstimation::run(int argc, char **argv) {
  ros::init (argc, argv, "object_position_estimation_server");
  ros::NodeHandle n;

  pListener = new (tf::TransformListener);

  service = n.advertiseService("object_position_estimation", &ObjectPositionEstimation::estimatePosition, this);
  //pub_position = n.advertise<ras_object_detection::PointStampedArray> ("/position", 1);

  ros::spin();
}

/*----------------------------------------------------------------------------*/

int main (int argc, char **argv) {
  ObjectPositionEstimation detector = ObjectPositionEstimation();
  detector.run(argc, argv);
}
