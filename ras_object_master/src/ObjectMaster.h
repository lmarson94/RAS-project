#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <object_detection/DetectObjectsImage.h>
#include <object_detection/msg_objects_detected.h>

#include <ras_object_detection/EstimateObjectPosition.h>
#include <ras_object_detection/PointStampedArray.h>
#include "sensor_msgs/PointCloud2.h"
#include <vector>

#include <geometry_msgs/PointStamped.h>

#include <ras_object_classification/ClassifyImage.h>
#include <ras_object_master/ObjectMasterMsg.h>

#include <ras_msgs/RAS_Evidence.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <sstream>

// #include <boost/make_shared.hpp>
// #include "compressed_image_transport/compressed_publisher.h"
// #include "compressed_image_transport/compression_common.h"
// namespace enc = sensor_msgs::image_encodings;
// using namespace std;
// using namespace cv;

class ObjectMaster
{
	public:
		ObjectMaster();
		void run(int argc, char **argv);
	private:
    		int updateFrequency;
		cv_bridge::CvImagePtr opencv_currentImage_ptr;
		cv::Mat currentOpencvImage;
		//subscribed topics
		image_transport::Subscriber imageSub;
		ros::Subscriber pclSub;

		//service clients
		ros::ServiceClient detectionClient;
		ros::ServiceClient positionClient;
		ros::ServiceClient classificationClient;

		//functions
		void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
		void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); //to be implemented
		void callServices();
		void resetFlow();
		uint16_t* createROI(uint16_t ulx, uint16_t uly, uint16_t lrx, uint16_t lry);
		void createResultImage(int resultArr[], int size);
		cv_bridge::CvImagePtr imageToCv(const sensor_msgs::ImageConstPtr& img_msg);
		std::string mapClassObject(int clasId);
		std::string mapColorObject(int colorId);
		int getObjectColorId(int clasId);
		//sensor_msgs::CompressedImage imageToCompressed(const sensor_msgs::Image& message); //adopted from compressed_image_transport

		//variables
		sensor_msgs::Image currentImage;
		sensor_msgs::PointCloud2 currentPcl;

		//topic out
		ros::Publisher pub_master;
		ros::Publisher pub_audio;
		ros::Publisher pub_evidence;
		image_transport::Publisher pub_resultImage;

		bool isImageReceived;	//booleans are used to create an ordered workflow and to
		bool isPclReceived;	//ensure data consistency / rough synchronisation
		bool isReceivingEnabled;
};
