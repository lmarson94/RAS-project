#include "SimpleBatteryPCLDetection.h"


SimpleBatteryPCLDetection::SimpleBatteryPCLDetection() {
	scene = PointCloudT::Ptr(new PointCloudT);
	currentCallbackIndex = 1;
	callbackWait = 1;
}

void SimpleBatteryPCLDetection::pclAlignmentCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	currentCallbackIndex += 1;
	currentTimeStamp = msg->header.stamp;
	if (currentCallbackIndex % callbackWait == 0){
		ROS_INFO("Analysing PCL");
		currentCallbackIndex = 0;
	}
	else{
		return;
	}
	//Transformation does not work because of rosbag --- fix later.
	//Transform to current robot position
	//sensor_msgs::PointCloud2 msg_pc2 = *msg;
	//sensor_msgs::PointCloud2 msg_converted;
	//std::string frame_base_link = "base_link";
	//pcl_ros::transformPointCloud(frame_base_link, msg_pc2, msg_converted, *pListener);


	//Convert from PCLPointCloud2 to PointCloud
	pcl::PCLPointCloud2* c = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr c_ptr(c);
	pcl_conversions::toPCL(*msg, *c);
	pcl::fromPCLPointCloud2(*c, *scene);

	float x, y, z, roll, pitch, yaw;
	x = 0.0925f; y = 0.0f; z = 0.18f;
	yaw = -1.5708f; pitch = 0.0f; roll = -1.9199f;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << x, y, z;
	transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
	transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

	pcl::transformPointCloud(*scene, *scene, transform);

	PointCloudT::Ptr scene_temp (new PointCloudT); // temp var for preprocessing steps
	//Downsample
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(scene);
	grid.filter(*scene_temp);

	//Remove NaN values
	std::vector<int> ind;
	pcl::removeNaNFromPointCloud(*scene_temp, *scene, ind);
	pcl::removeNaNFromPointCloud(*scene_temp, *scene_temp, ind); // only for debugging
	ROS_INFO("Size after (Scene): %d", scene->points.size());

	//Normal - necessary?
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  	nest.setRadiusSearch (0.01);
  	nest.setInputCloud (scene);
  	nest.compute(*scene);

	//Remove walls, remove floor, remove points far away

	//Slicing is now done after clusterisation
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<PointNT> extract;

	float zOff_neg = 0.035f;
	float zOff_pos = 0.5f;
 	float yOff_neg = -0.40f;
	float yOff_pos = 0.40f;
	float xOff = 1.5f;

	for (int i = 0; i < scene->size(); i++){
		pcl::PointXYZ pt(scene->points[i].x, scene->points[i].y, scene->points[i].z);
		if (!((pt.z > zOff_neg) && (pt.z < zOff_pos))){ //5 cm above ground
			inliers->indices.push_back(i);
		}
		else if (!((pt.y > yOff_neg) && (pt.y < yOff_pos))){ //sides
			inliers->indices.push_back(i);
		}
		else if (pt.x > xOff){ //far away
			inliers->indices.push_back(i);
		}
		else{
			continue;
		}
	}

	ROS_INFO("Start Filtering");
	extract.setInputCloud(scene);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*scene);

	//Detect Clusters
	pcl::search::KdTree<PointNT>::Ptr tree (new pcl::search::KdTree<PointNT>);
	tree->setInputCloud(scene);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointNT> ec;

	ec.setClusterTolerance(0.05);

	ec.setMinClusterSize(30); // ??

	ec.setMaxClusterSize(30000); // set to 100 ?
	ec.setSearchMethod(tree);
	ec.setInputCloud(scene)	;
	ec.extract(cluster_indices);
	//pcl::visualization::PCLVisualizer visu("Alignment");
	int i = 0;
	int size = 0;
	bool isVert = false;
	bool basicCheck = false;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
		pcl::PointCloud<PointNT>::Ptr cloud_cluster (new pcl::PointCloud<PointNT>);
		//find interesting region with right amount of points
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			cloud_cluster->points.push_back(scene->points[*pit]);
		}

		basicCheck = basicBatteryCheck(cloud_cluster);
		size = cloud_cluster->points.size();
		ROS_INFO("size: %d", size);
		if (size > 30 && size < 800 && basicCheck){

			ROS_INFO("Found Battery Cluster");
			pcl::PointCloud<PointNT>::Ptr upperupper_cluster (new pcl::PointCloud<PointNT>);
			pcl::PointCloud<PointNT>::Ptr upper_cluster (new pcl::PointCloud<PointNT>);
			pcl::PointCloud<PointNT>::Ptr lower_cluster (new pcl::PointCloud<PointNT>);
			//Now do slicing here
			pcl::PointIndices::Ptr inliers_t(new pcl::PointIndices());
			pcl::ExtractIndices<PointNT> extract_t;
			float zOff_neg_t = 0.035f;
			float zOff_pos_t = 0.045f;
			for (i = 0; i < cloud_cluster->size(); i++){
				pcl::PointXYZ pt(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z);
				if (!((pt.z > zOff_neg_t) && (pt.z < zOff_pos_t))){ //slice 5 cm above ground
					inliers_t->indices.push_back(i);
				}
			}

			ROS_INFO("Start Slicing Lower Part");
			extract_t.setInputCloud(cloud_cluster);
			extract_t.setIndices(inliers_t);
			extract_t.setNegative(true);
			extract_t.filter(*lower_cluster);

			lower_cluster->width = lower_cluster->points.size();
			lower_cluster->height = 1;
			lower_cluster->is_dense = true;


			/*** italian swear word dictionary ***/

			//vaffanculo
			//che palle

			/***				   ***/


			//Do second slicing at 11 cm
			pcl::PointIndices::Ptr inliers_u(new pcl::PointIndices());
			pcl::ExtractIndices<PointNT> extract_u;
			float zOff_neg_u = 0.105f;
			float zOff_pos_u = zOff_neg_u + 0.01f;
			for (i = 0; i < cloud_cluster->size(); i++){
				pcl::PointXYZ pt(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z);
				if (!((pt.z > zOff_neg_u) && (pt.z < zOff_pos_u))){ //slice 11 cm above ground
					inliers_u->indices.push_back(i);
				}
			}


			ROS_INFO("Start Slicing Upper Part");
			extract_u.setInputCloud(cloud_cluster);
			extract_u.setIndices(inliers_u);
			extract_u.setNegative(true);
			extract_u.filter(*upper_cluster);

			upper_cluster->width = upper_cluster->points.size();
			upper_cluster->height = 1;
			upper_cluster->is_dense = true;

			if (lower_cluster->points.size() < 10 && upper_cluster->points.size() < 10){
				ROS_INFO("Aborted - detected as noise");
				return;
			}

			if(upper_cluster->points.size() > 15){
				ROS_INFO("Vertical position");
				isVert = true;
				//Calculate max and min y
				pcl::PointXYZ max_y;
				max_y.y = -10;
				pcl::PointXYZ min_y;
				min_y.y = 10;

				for (int i = 0; i < upper_cluster->size(); i++){
					pcl::PointXYZ pt(upper_cluster->points[i].x, upper_cluster->points[i].y, upper_cluster->points[i].z);
					if (pt.y < min_y.y)
						min_y = pt;
					else if (pt.y > max_y.y)
						max_y = pt;
					else
						continue;
				}
				ROS_INFO("min y: %f %f %f", min_y.x, min_y.y, min_y.z);
				ROS_INFO("max y: %f %f %f", max_y.x, max_y.y, max_y.z);
				if (fabs(fabs(max_y.y)-fabs(min_y.y)) > 0.11)
				{
					ROS_INFO("Width to large for vertical position.");
					return;
				}
			}
			else{
				ROS_INFO("Horizontal position");
				isVert = false;
			}

			//Do third slicing at 17 cm
			pcl::PointIndices::Ptr inliers_uu(new pcl::PointIndices());
			pcl::ExtractIndices<PointNT> extract_uu;
			float zOff_neg_uu = 0.165f;
			float zOff_pos_uu = 0.175f;
			for (i = 0; i < cloud_cluster->size(); i++){
				pcl::PointXYZ pt(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z);
				if (!((pt.z > zOff_neg_uu) && (pt.z < zOff_pos_uu))){ //slice 17 cm above ground
					inliers_uu->indices.push_back(i);
				}
			}

			ROS_INFO("Start Slicing Upper Upper Part");
			extract_uu.setInputCloud(cloud_cluster);
			extract_uu.setIndices(inliers_uu);
			extract_uu.setNegative(true);
			extract_uu.filter(*upperupper_cluster);

			upperupper_cluster->width = upperupper_cluster->points.size();
			upperupper_cluster->height = 1;
			upperupper_cluster->is_dense = true;

			if(upperupper_cluster->points.size() > 5){
				ROS_INFO("Third battery check: proof failed");
				return;
			}
			else{
				ROS_INFO("Third battery check: success");
			}

			//battery_cluster = lower_cluster; //not necessary.
			if (isVert)
				calcBatteryCenter(scene, upper_cluster, isVert);
			else
				calcBatteryCenter(scene, lower_cluster, isVert);

		}

		//visu.addPointCloud (cloud_cluster, ColorHandlerT (cloud_cluster, 255/i, 0.0, 0.0), "scene");
	}


	//visu.addPointCloud (battery_cluster, ColorHandlerT (battery_cluster, 255, 255, 0.0), "battery_cluster");

	//visu.spin();
	return;
}

bool SimpleBatteryPCLDetection::basicBatteryCheck(const pcl::PointCloud<PointNT>::Ptr cloud){

	//Calculate max and min x
	pcl::PointXYZ max_x;
	max_x.x = -10;
	pcl::PointXYZ min_x;
	min_x.x = 10;

	for (int i = 0; i < cloud->size(); i++){
		pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		if (pt.x < min_x.x)
			min_x = pt;
		else if (pt.x > max_x.x)
			max_x = pt;
		else
			continue;
	}
	ROS_INFO("min x: %f %f %f", min_x.x, min_x.y, min_x.z);
	ROS_INFO("max x: %f %f %f", max_x.x, max_x.y, max_x.z);

	//Calculate max and min y
	pcl::PointXYZ max_y;
	max_y.y = -10;
	pcl::PointXYZ min_y;
	min_y.y = 10;

	for (int i = 0; i < cloud->size(); i++){
		pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		if (pt.y < min_y.y)
			min_y = pt;
		else if (pt.y > max_y.y)
			max_y = pt;
		else
			continue;
	}
	ROS_INFO("min y: %f %f %f", min_y.x, min_y.y, min_y.z);
	ROS_INFO("max y: %f %f %f", max_y.x, max_y.y, max_y.z);

	//Calculate max and min z
	pcl::PointXYZ max_z;
	max_z.z = -10;
	pcl::PointXYZ min_z;
	min_z.z = 10;

	for (int i = 0; i < cloud->size(); i++){
		pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		if (pt.z < min_z.z)
			min_z = pt;
		else if (pt.z > max_z.z)
			max_z = pt;
		else
			continue;
	}
	ROS_INFO("min z: %f %f %f", min_z.x, min_z.y, min_z.z);
	ROS_INFO("max z: %f %f %f", max_z.x, max_z.y, max_z.z);

	bool result = true;

	float area = fabs((fabs(max_y.y)-fabs(min_y.y))*(fabs(max_z.z)-fabs(min_z.z)));
	ROS_INFO("AREA %f", area);

	if (fabs(fabs(max_x.x)-fabs(min_x.x)) > 0.16)
		result = false;
	else if (fabs(fabs(max_y.y)-fabs(min_y.y)) > 0.16)
		result = false;
	else if (fabs(fabs(max_z.z)-fabs(min_z.z)) > 0.15)
		result = false;
	else if (area > 0.008)
		result = false;
	else if (min_x.x > 1.0)
		result = false;
	else
		ROS_INFO("Basic test pass");

	return result;

}

void SimpleBatteryPCLDetection::calcBatteryCenter(const pcl::PointCloud<PointNT>::Ptr sce, const pcl::PointCloud<PointNT>::Ptr bc, bool isVertical){

	pcl::PointXYZ max_y;
	max_y.y = -10;
	pcl::PointXYZ min_y;
	min_y.y = 10;

	for (int i = 0; i < bc->size(); i++){
		pcl::PointXYZ pt(bc->points[i].x, bc->points[i].y, bc->points[i].z);
		if (pt.y < min_y.y)
			min_y = pt;
		else if (pt.y > max_y.y)
			max_y = pt;
		else
			continue;
	}
	ROS_INFO("min y: %f %f %f", min_y.x, min_y.y, min_y.z);
	ROS_INFO("max y: %f %f %f", max_y.x, max_y.y, max_y.z);

	if (fabs(fabs(min_y.y)-fabs(max_y.y)) < 0.03 || fabs(fabs(min_y.y)-fabs(max_y.y)) > 0.165){
		ROS_INFO("Last Checkpoint 1: FAIL !!");
		return;
	}


	pcl::PointXYZ centerPoint;
	bool isLeftPoint = false;
	if (max_y.x < min_y.x){
		centerPoint = max_y;
		isLeftPoint = false;
	}
	else{
		centerPoint = min_y;
		isLeftPoint = true;
	}

	centerPoint.x = centerPoint.x + 0.035;

	if (isLeftPoint){ // then go to the right...
		centerPoint.y = centerPoint.y + fabs(max_y.y - min_y.y) / 2.0;
	}
	else{
		centerPoint.y = centerPoint.y - fabs(min_y.y - max_y.y) / 2.0;
	}
	ROS_INFO("CenterPoint: %f %f %f", centerPoint.x, centerPoint.y, centerPoint.z);

	/*** ONLY FOR DEBUGGING ***/
	/**
	pcl::visualization::PCLVisualizer visu("Points of Interest");
	visu.addPointCloud (sce, ColorHandlerT (sce, 0.0, 0.0, 255.0), "scene");
	visu.addPointCloud (bc, ColorHandlerT (bc, 255, 255, 0.0), "battery_cluster");
	pcl::PointCloud<PointNT>::Ptr interestCloud (new pcl::PointCloud<PointNT>);
	interestCloud->width = 3;
	interestCloud->height = 1;
	interestCloud->is_dense = true;
	interestCloud->resize(interestCloud->width*interestCloud->height);
	interestCloud->points[0].x = min_y.x;
	interestCloud->points[0].y = min_y.y;
	interestCloud->points[0].z = min_y.z;
	interestCloud->points[1].x = max_y.x;
	interestCloud->points[1].y = max_y.y;
	interestCloud->points[1].z = max_y.z;
	interestCloud->points[2].x = centerPoint.x;
	interestCloud->points[2].y = centerPoint.y;
	interestCloud->points[2].z = centerPoint.z;
	visu.addPointCloud (interestCloud, ColorHandlerT (interestCloud, 255.0, 0.0, 0.0), "interest_cloud");
	visu.spin();
	**/
	/*** 		***/

	geometry_msgs::PointStamped pointCamera, pointMap;
	pointCamera.header.frame_id = "base_link";
	pointCamera.header.stamp = currentTimeStamp;
	pointCamera.point.x = centerPoint.x;
	pointCamera.point.y = centerPoint.y;
	pointCamera.point.z = centerPoint.z;


	pListener -> transformPoint("base_map", pointCamera, pointMap);

	geometry_msgs::Point resultPoint;
	resultPoint.x = pointMap.point.x;
	resultPoint.y = pointMap.point.y;
	if (isVertical)
		resultPoint.z = 1.0f;
	else
		resultPoint.z = 2.0f;

	posPub.publish(resultPoint);

	// posPub.publish(pointCamera); //delete later
	return;

}

void SimpleBatteryPCLDetection::detect(int argc, char **argv) {

	n = ros::NodeHandle("~");
	pListener = new (tf::TransformListener);

  posPub = n.advertise<geometry_msgs::Point> ("/battery/Geometry/New_Point", 1); //change topic to /Geometry/New_Point for directly using the map or change topic to battery/Geometry/New_Point to use the battery decision master in between.
	pclSub = n.subscribe("/camera/depth_registered/points", 1, &SimpleBatteryPCLDetection::pclAlignmentCallback, this);
	pclPub = n.advertise<sensor_msgs::PointCloud2>("/test_pcl_output", 1);

	ros::Rate lr(2);

	while(ros::ok())
	{
    		ros::spinOnce();
    		lr.sleep();
	}
}


int main (int argc, char **argv) {
	ros::init (argc, argv, "battery_detection");
  	SimpleBatteryPCLDetection detector = SimpleBatteryPCLDetection();
  	detector.detect(argc, argv);
}
