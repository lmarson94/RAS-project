#include "ObjectMaster.h"

ObjectMaster::ObjectMaster()
{
	updateFrequency = 10; // ???
	isImageReceived = false;
	isPclReceived = false;
	isReceivingEnabled = true;
}

void ObjectMaster::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	if (isImageReceived == false & isReceivingEnabled == true){
		currentImage = *msg;
		opencv_currentImage_ptr = imageToCv(msg);
		(opencv_currentImage_ptr->image).copyTo(currentOpencvImage);
		ROS_INFO("Received an image.");
		ROS_INFO("%d %d", msg->header.stamp.sec, msg->header.stamp.nsec);
		isImageReceived = true;
	}

	/** DEBUG_INFO **/
	// sensor_msgs::Image img_msg;
	// std_msgs::Header header = currentImage.header;
	// cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, currentOpencvImage);
	// img_bridge.toImageMsg(img_msg);
	// pub_resultImage.publish(img_msg);

	return;
}
void ObjectMaster::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	if (isPclReceived == false & isImageReceived == true & isReceivingEnabled == true){
		currentPcl = *msg;
		ROS_INFO("Received a pcl.");
		isPclReceived = true;
		isReceivingEnabled = false;ros::Time begin = ros::Time::now();
                ROS_INFO("%d %d", msg->header.stamp.sec, msg->header.stamp.nsec);
		callServices();
	}
	return;
}
cv_bridge::CvImagePtr ObjectMaster::imageToCv(const sensor_msgs::ImageConstPtr& img_msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		cv_ptr = NULL;
	}
	return cv_ptr;
}
uint16_t* ObjectMaster::createROI(uint16_t ulx, uint16_t uly, uint16_t lrx, uint16_t lry)
{
		/**
		uint16_t area_ex = 30;
		uint16_t dim[4] = {0, 0, 639, 479};
		if (ulx - area_ex > 0)
			dim[0] = ulx - area_ex;
		if (uly - area_ex > 0)
			dim[1] = uly - area_ex;
		if (lrx + area_ex < 640)
			dim[2] = lrx + area_ex;
		if (lry + area_ex < 480)
			dim[3] = lry + area_ex;
		uint16_t *roi = new uint16_t[4];
		roi[0] = dim[0];
		roi[1] = dim[1];
		roi[2] = dim[2]-dim[0];
		roi[3] = dim[3]-dim[1];
		return roi;
		**/

		float multiplier = 1.3f;
		uint16_t area_x = uint16_t(float(lrx-ulx)*multiplier - float(lrx-ulx));
		uint16_t area_y = uint16_t(float(lry-uly)*multiplier - float(lry-uly));

		uint16_t dim[4] = {0, 0, 639, 479};
		if (ulx - area_x > 0)
			dim[0] = ulx - area_x;
		if (uly - area_y > 0)
			dim[1] = uly - area_y;
		if (lrx + area_x < 640)
			dim[2] = lrx + area_x;
		if (lry + area_y < 480)
			dim[3] = lry + area_y;
		uint16_t *roi = new uint16_t[4];
		roi[0] = dim[0];
		roi[1] = dim[1];
		roi[2] = dim[2]-dim[0];
		roi[3] = dim[3]-dim[1];

		return roi;

}
void ObjectMaster::callServices()
{
	//Call Service for object detection:
	object_detection::DetectObjectsImage srv;
	sensor_msgs::Image img_msg;
	std_msgs::Header header = currentImage.header;
	cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, currentOpencvImage);
	img_bridge.toImageMsg(img_msg);
	srv.request.inputImage = img_msg;
	object_detection::msg_objects_detected ansObjectsDetected;
	if(detectionClient.call(srv)){
		ansObjectsDetected = srv.response.detectionOut;
		ROS_INFO("Objects detected.");
		ROS_INFO("%d", ansObjectsDetected.coords.size()/7);
	}

	//Call Position Estimation service:
	ras_object_detection::EstimateObjectPosition srv2;
	srv2.request.inputCloud = currentPcl;
	srv2.request.objectList = ansObjectsDetected;
	ras_object_detection::PointStampedArray ansObjectsPositions;
	ros::service::waitForService("/object_position_estimation", 500);
	if(positionClient.call(srv2)){
		ansObjectsPositions = srv2.response.positions;
		ROS_INFO("Position detected.");
		ROS_INFO("%d", ansObjectsPositions.points.size());
	}
	//I am hopefully assuming that the positions are in the same order like the bounding boxes from the detection.
	ROS_INFO("%d", ansObjectsDetected.coords.size());
	ROS_INFO("%d", ansObjectsPositions.points.size());
	if (ansObjectsDetected.coords.size()/7 != ansObjectsPositions.points.size())
	{
		ROS_ERROR("IDENTITY MISMATCH!");
		resetFlow();
		return;
	}
	int resultsSize = ansObjectsDetected.coords.size();
	int results[resultsSize];//bb1, bb2, bb3, bb4, clas, conf, color
	for ( int i = 0; i < ansObjectsDetected.coords.size()/7; i++ )
	{
		if (ansObjectsPositions.points[i].point.x == -99) //no classification necessary if object pos. could not be estimated.
		{
			uint16_t *curROI = createROI(ansObjectsDetected.coords[i*7+0], ansObjectsDetected.coords[i*7+1], ansObjectsDetected.coords[i*7+2], ansObjectsDetected.coords[i*7+3]);
			results[i*7+0] = curROI[0];
			results[i*7+1] = curROI[1];
			results[i*7+2] = curROI[2] + curROI[0];
			results[i*7+3] = curROI[3] + curROI[1];
			results[i*7+4] = 15; //clas
			results[i*7+5] = -99;
			results[i*7+6] = ansObjectsDetected.coords[i*7+6];
			continue;
		}
		ras_object_classification::ClassifyImage srv3;
		uint16_t *curROI = createROI(ansObjectsDetected.coords[i*7+0], ansObjectsDetected.coords[i*7+1], ansObjectsDetected.coords[i*7+2], ansObjectsDetected.coords[i*7+3]);
		/** results arr **/
		//results[i*7+0] = ansObjectsDetected.coords[i*7+0]; //bb1
		//results[i*7+1] = ansObjectsDetected.coords[i*7+1]; //bb2
		//results[i*7+2] = ansObjectsDetected.coords[i*7+2]; //bb3
		//results[i*7+3] = ansObjectsDetected.coords[i*7+3]; //bb4
		results[i*7+6] = ansObjectsDetected.coords[i*7+6]; //color

		cv::Mat croppedImage;
		currentOpencvImage.copyTo(croppedImage);
		ROS_INFO("ROI: %d %d %d %d", curROI[0], curROI[1], curROI[2], curROI[3]);
		results[i*7+0] = curROI[0];
		results[i*7+1] = curROI[1];
		results[i*7+2] = curROI[2] + curROI[0];
		results[i*7+3] = curROI[3] + curROI[1];
		//cv::Mat cropImg(croppedImage, cv::Rect(*(curROI+0), *(curROI+1), *(curROI+2), *(curROI+3)));
		cv::Mat cropImg = croppedImage(cv::Rect(*(curROI+0), *(curROI+1), *(curROI+2), *(curROI+3)));
		sensor_msgs::Image img_msg;
		std_msgs::Header header = currentImage.header;
		cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cropImg);
		img_bridge.toImageMsg(img_msg);

		//cropImgPtr->image = cropImg;
		//cropImg.copyTo(cropImgPtr->image);
		srv3.request.inputImage = img_msg;
		ros::service::waitForService("/classify_image", 20000);
		if(classificationClient.call(srv3)){
			int conf = srv3.response.conf;
			int clas = srv3.response.clas;
			ROS_INFO("Classified Object, Clas: %d, Conf: %d", clas, conf);

			//after classification of an object is done, its position is sent out via topic provided by the master
			ras_object_master::ObjectMasterMsg msg_out;
			msg_out.point = ansObjectsPositions.points[i];
			msg_out.clas = clas;
			msg_out.conf = conf;

			//record evidence
			ras_msgs::RAS_Evidence msg_ev;

			//send message to the speaker
			//std_msgs::String msg_audio;
			if (getObjectColorId(clas) != results[i*7+6]){ // if confidence is too low, just say that you see an object.
					//or maybe if the object is too far away, do not send out the classification.
				//msg_audio.data = "I see " + ras_msgs::RAS_Evidence::an_object;

				ROS_INFO("Color mismatch");
				//Except cases: blue cube and green cube
				if (conf > 85 && clas == 11 && results[i*7+6] == 5){ // test this !!!
					msg_ev.object_id = ras_msgs::RAS_Evidence::green_cube;
					msg_out.clas = 3; //turns class blue cube and color green to result: green cube
				}
				else if (conf > 85 && clas == 3 && results[i*7+6] == 2){ // test this !!!
					msg_ev.object_id = ras_msgs::RAS_Evidence::blue_cube;
					msg_out.clas = 11; //turns class green cube and color blue to result: blue cube
				}
				else{
					msg_ev.object_id = ras_msgs::RAS_Evidence::an_object;
					msg_out.clas = 15;
				}
			}
			else{
				//msg_audio.data = "I see a " + mapClassObject(clas);
				msg_ev.object_id = mapClassObject(clas);
			}
			//pub_audio.publish(msg_audio);

			/** results arr **/
			results[i*7+4] = msg_out.clas; //clas
			results[i*7+5] = msg_out.conf; //conf

			//pack evidence message
			msg_ev.image_evidence = img_msg;
			msg_ev.group_number = 1;
			msg_ev.stamp = ros::Time::now();
			msg_ev.object_location.transform.translation.x = ansObjectsPositions.points[i].point.x;
			msg_ev.object_location.transform.translation.y = ansObjectsPositions.points[i].point.y;

			//publish evidence message.
			pub_evidence.publish(msg_ev);

			//Publish master output message
			pub_master.publish(msg_out);

		}

	}

	createResultImage(results, resultsSize);
	resetFlow();

	return;

}
void ObjectMaster::createResultImage(int resultArr[], int size){
	for (int i = 0; i < int(size/7); i++){
		cv::Point p1(resultArr[i*7+0], resultArr[i*7+1]);
		cv::Point p2(resultArr[i*7+2], resultArr[i*7+3]);

		cv::rectangle(currentOpencvImage, p1, p2, cv::Scalar(0, 255, 0));

		std::string color = mapColorObject(resultArr[i*7+6]);
		std::string object = mapClassObject(resultArr[i*7+4]);

		cv::Point p3(resultArr[i*7+0], resultArr[i*7+3] + 15);
		cv::Point p4(resultArr[i*7+0], resultArr[i*7+3] + 30);
		cv::putText(currentOpencvImage, object, p3, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, CV_AA);
		cv::putText(currentOpencvImage, color, p4, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, CV_AA);
	}
	sensor_msgs::Image img_msg;
	std_msgs::Header header = currentImage.header;
	cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, currentOpencvImage);
	img_bridge.toImageMsg(img_msg);
	pub_resultImage.publish(img_msg);

}

int ObjectMaster::getObjectColorId(int clasId){
	int map[14] = {4, 4, 5, 5, 5, 1, 1, 3, 3, 3, 2, 2, 0, 0};
	return map[clasId-1];
}

std::string ObjectMaster::mapColorObject(int colorId){
	std::string map[6] = {"purple", "orange", "blue", "red", "yellow", "green"};
	return map[colorId];

}
std::string ObjectMaster::mapClassObject(int clasId) // maps the classification result to a corresponding String, describing the object
{
	//ras_msgs::RAS_Evidence msg;
	std::string map[15] = {ras_msgs::RAS_Evidence::yellow_ball, ras_msgs::RAS_Evidence::yellow_cube, ras_msgs::RAS_Evidence::green_cube, ras_msgs::RAS_Evidence::green_cylinder, ras_msgs::RAS_Evidence::green_hollow_cube, ras_msgs::RAS_Evidence::orange_cross, ras_msgs::RAS_Evidence::patric, ras_msgs::RAS_Evidence::red_cylinder, ras_msgs::RAS_Evidence::red_hollow_cube, ras_msgs::RAS_Evidence::red_ball, ras_msgs::RAS_Evidence::blue_cube, ras_msgs::RAS_Evidence::blue_triangle, ras_msgs::RAS_Evidence::purple_cross, ras_msgs::RAS_Evidence::purple_star, ras_msgs::RAS_Evidence::an_object};
	return map[clasId-1];
}
void ObjectMaster::resetFlow()
{
	isImageReceived = false;
	isPclReceived = false;
	isReceivingEnabled = true;
	ROS_INFO("Flow Reset");
}
void ObjectMaster::run(int argc, char **argv)
{

	ros::init(argc, argv, "ObjectMaster");
	ros::NodeHandle n("~");
	ros::Rate loop_rate(updateFrequency);
	//ROS_INFO_STREAM(mapClassObject(1));
	image_transport::ImageTransport it(n);

	imageSub = it.subscribe("/camera/rgb/image_rect_color", 1, &ObjectMaster::imageCallback, this, image_transport::TransportHints("compressed")); // this -> ros::VoidPtr(), image_transport::TransportHints("compressed")
	pclSub = n.subscribe("/camera/depth_registered/points", 1, &ObjectMaster::pclCallback, this);

	detectionClient = n.serviceClient<object_detection::DetectObjectsImage>("/detect_objects_image");

	positionClient = n.serviceClient<ras_object_detection::EstimateObjectPosition>("/object_position_estimation");

	classificationClient = n.serviceClient<ras_object_classification::ClassifyImage>("/classify_image");

	pub_master = n.advertise<ras_object_master::ObjectMasterMsg> ("/object_master_out", 1);
	pub_audio = n.advertise<std_msgs::String> ("/espeak/string", 1);
	pub_evidence = n.advertise<ras_msgs::RAS_Evidence> ("/evidence", 1);
	pub_resultImage = it.advertise("/object_master_out/result_image", 1);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return;
}
/**
sensor_msgs::CompressedImage ObjectMaster::imageToCompressed(const sensor_msgs::Image& message){

	namespace enc = sensor_msgs::image_encodings;

	// Compressed image message
	sensor_msgs::CompressedImage compressed;
	compressed.header = message.header;
	compressed.format = message.encoding;

	// Compression settings
 std::vector<int> params;
 params.resize(3, 0);

 // Get codec configuration
 int encFormat = 0;

 // Bit depth of image encoding
 int bitDepth = enc::bitDepth(message.encoding);
 int numChannels = enc::numChannels(message.encoding);

 switch (encFormat)
 {
   // JPEG Compression
   case 0:
   {
     params[0] = CV_IMWRITE_JPEG_QUALITY;
     //params[1] = config_.jpeg_quality;
		 params[1] = 0.8;

     // Update ros message format header
     compressed.format += "; jpeg compressed ";

     // Check input format
     if ((bitDepth == 8) || (bitDepth == 16))
     {
       // Target image format
       std::string targetFormat;
       if (enc::isColor(message.encoding))
       {
         // convert color images to BGR8 format
         targetFormat = "bgr8";
         compressed.format += targetFormat;
       }

       // OpenCV-ros bridge
       try
       {
				 cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, "bgr8");

         // Compress image
         if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
         {

           float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
               / (float)compressed.data.size();
           ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
         }
         else
         {
           ROS_ERROR("cv::imencode (jpeg) failed on input image");
         }
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("%s", e.what());
       }
       catch (cv::Exception& e)
       {
         ROS_ERROR("%s", e.what());
       }

       // Publish message
      return compressed;
     }
     else
       ROS_ERROR("Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)", message.encoding.c_str());

     break;
   }
     // PNG Compression
   case 1:
   {
     params[0] = CV_IMWRITE_PNG_COMPRESSION;
     //params[1] = config_.png_level;
		 params[1] = 0.8;
     // Update ros message format header
     compressed.format += "; png compressed ";

     // Check input format
     if ((bitDepth == 8) || (bitDepth == 16))
     {

       // Target image format
       stringstream targetFormat;
       if (enc::isColor(message.encoding))
       {
         // convert color images to RGB domain
         targetFormat << "bgr" << bitDepth;
         compressed.format += targetFormat.str();
       }
			 std::string e = std::string();
       // OpenCV-ros bridge
       try
       {
         cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, e);

         // Compress image
         if (cv::imencode(".png", cv_ptr->image, compressed.data, params))
         {

           float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
               / (float)compressed.data.size();
           ROS_DEBUG("Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
         }
         else
         {
           ROS_ERROR("cv::imencode (png) failed on input image");
         }
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("%s", e.what());
         return compressed;
       }
       catch (cv::Exception& e)
       {
         ROS_ERROR("%s", e.what());
         return compressed;
       }

       // Publish message
			 return compressed;
     }
     else
       ROS_ERROR("Compressed Image Transport - PNG compression requires 8/16-bit encoded color format (input format is: %s)", message.encoding.c_str());
     break;
   }

   default:
     //ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'", config_.format.c_str());
     break;
 }

	return compressed;
}
**/
int main(int argc, char **argv)
{
	ObjectMaster objectMaster = ObjectMaster();
	objectMaster.run(argc, argv);
}
