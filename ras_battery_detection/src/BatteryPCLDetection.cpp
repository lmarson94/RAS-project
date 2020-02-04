#include "BatteryPCLDetection.h"


BatteryPCLDetection::BatteryPCLDetection() {
	object = PointCloudT::Ptr (new PointCloudT);
	object_aligned = PointCloudT::Ptr (new PointCloudT);
	scene = PointCloudT::Ptr(new PointCloudT);
	object_features = FeatureCloudT::Ptr(new FeatureCloudT);
	scene_features = FeatureCloudT::Ptr(new FeatureCloudT);

	pcl::PointCloud<PointNT>::Ptr temp (new pcl::PointCloud<PointNT>);

	if (pcl::io::loadPCDFile<PointNT> ("templateMUnit.pcd", *object) == -1)
	{ //provides: x y z normal_x normal_y normal_z

		PCL_ERROR("Couldn't read file");
	}
	std::cout << "Loaded " 
			<< object->width * object->height
			<< " data points from file with the following fields"
			<< std::endl;
	for (size_t i = 0; i < int(object->points.size()/100); ++i) //only print first lines
	{
		std::cout << " " << object->points[i].x
				<< " " << object->points[i].y
				<< " " << object->points[i].z << std::endl;
	}

	//temp_cloud = new pcl::PCLPointCloud2; //only for debugging
	//pcl::toPCLPointCloud2(*object, *temp_cloud); //only for debugging

	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.01f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);
	ROS_INFO("Size of Object: %d", object->points.size());

}

void BatteryPCLDetection::pubTemplate(const sensor_msgs::PointCloud2ConstPtr& msg){ //only used for debugging
	ROS_INFO("#######Template#########");

	//Convert
	sensor_msgs::PointCloud2 output;
	//sensor_msgs::convertPointCloudToPointCloud2(*temp_ptr, output);
	//pcl::toPCLPointCloud2(temp_cloud, output);
	pcl_conversions::fromPCL(*temp_cloud, output);
	//Publish
	output.header.frame_id = "map";
	pclPub.publish(output);
	return;
}
void BatteryPCLDetection::pclAlignmentCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	ROS_INFO("#######Alignment#########");	
	
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
	const float leaf = 0.01f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(scene);
	grid.filter(*scene_temp);

	//Remove NaN values
	std::vector<int> ind;
	pcl::removeNaNFromPointCloud(*scene_temp, *scene, ind);
	pcl::removeNaNFromPointCloud(*scene_temp, *scene_temp, ind); // only for debugging
	ROS_INFO("Size after (Scene): %d", scene->points.size());
	ROS_INFO("Size after (Object): %d", object->points.size());
	
	//Remove walls, remove floor, remove points far away	
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<PointNT> extract;
	float zOff = 0.02f;
	float yOff_neg = -0.15f;
	float yOff_pos = 0.15f;
	float xOff = 1.0f;
	for (int i = 0; i < scene->size(); i++){
		pcl::PointXYZ pt(scene->points[i].x, scene->points[i].y, scene->points[i].z);
		if (pt.z < zOff){ //floor
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
	extract.setInputCloud(scene);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*scene);
	
	//Normal
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  	nest.setRadiusSearch (0.01);
  	nest.setInputCloud (scene);
  	nest.compute(*scene);

	//Features
	FeatureEstimationT fest;
	fest.setRadiusSearch (0.025);
	fest.setInputCloud (object);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);

	//Alignment Load Parameters
	int maxIter, noSamples, corRand;
	float simThres, maxDist, inFra;
	n.getParam("/battery_detect/MaximumIterations", maxIter);
	n.getParam("/battery_detect/NumberOfSamples", noSamples);
	n.getParam("/battery_detect/CorrespondenceRandomness", corRand);
	n.getParam("/battery_detect/SimilarityThreshold", simThres);
	n.getParam("/battery_detect/MaxCorrespondenceDistance", maxDist);
	n.getParam("/battery_detect/InlierFraction", inFra);
	
	//Alignment Calculation
	pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(maxIter);
	align.setNumberOfSamples(noSamples); 
	align.setCorrespondenceRandomness(corRand); 
	align.setSimilarityThreshold (simThres); 
	align.setMaxCorrespondenceDistance(maxDist); 
	Eigen::Matrix4f transformation_1;

	align.setInlierFraction (inFra);
	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}
	if (align.hasConverged())
	{
		ROS_INFO("1. Alignment SUCCESS!");
		transformation_1 = align.getFinalTransformation ();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_1 (0,0), transformation_1 (0,1), transformation_1 (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_1 (1,0), transformation_1 (1,1), transformation_1 (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_1 (2,0), transformation_1 (2,1), transformation_1 (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_1 (0,3), transformation_1 (1,3), transformation_1 (2,3));
		pcl::console::print_info ("\n");

		pcl::PointIndices::Ptr inliers_t(new pcl::PointIndices());
		pcl::ExtractIndices<PointNT> extract_t;
		float dist = 0.15f;		
		float zOff_neg_t = transformation_1 (2,3) - dist;
		float zOff_pos_t = transformation_1 (2,3) + dist;
		float yOff_neg_t = transformation_1 (1,3) - dist;
		float yOff_pos_t = transformation_1 (1,3) + dist;
		float xOff_neg_t = transformation_1 (0,3) - dist;
		float xOff_pos_t = transformation_1 (0,3) + dist;
		for (int i = 0; i < scene->size(); i++){
			pcl::PointXYZ pt(scene->points[i].x, scene->points[i].y, scene->points[i].z);
			if (!((pt.x > xOff_neg_t) && (pt.x < xOff_pos_t))){
				inliers_t->indices.push_back(i);
			} 
			else if (!((pt.y > yOff_neg_t) && (pt.y < yOff_pos_t))){
				inliers_t->indices.push_back(i);
			}
			else if (!((pt.z > zOff_neg_t) && (pt.z < zOff_pos_t))){
				inliers_t->indices.push_back(i);
			}  
			else{
				continue;
			}
		}
		extract_t.setInputCloud(scene);
		extract_t.setIndices(inliers_t);
		extract_t.setNegative(true);
		extract_t.filter(*scene);
		//pcl::visualization::PCLVisualizer visu("Alignment");
		//visu.addPointCloud (scene_temp, ColorHandlerT (scene_temp, 255.0, 255.0, 0.0), "scene_temp");
		//visu.addPointCloud (scene, ColorHandlerT (scene, 255.0, 0.0, 0.0), "scene");
		//visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		//visu.spin ();
	}
	else
	{
		ROS_INFO("1. Alignment FAIL!");
	}
	
	if (align.hasConverged()){
		//ICP 
		pcl::IterativeClosestPoint<PointNT, PointNT> icp;
		std::vector<int> index_t;
		pcl::removeNaNFromPointCloud(*object_aligned, *object_aligned, index_t);
		pcl::removeNaNFromPointCloud(*scene, *scene, index_t);
		icp.setInputSource(object_aligned);
		icp.setInputTarget(scene);
		PointCloudT::Ptr obj_align = PointCloudT::Ptr (new PointCloudT);
		icp.align(*obj_align);
		Eigen::Matrix4f transformation_2;
		if (icp.hasConverged())
		{
			ROS_INFO("2. Alignment SUCCESS!");
			ROS_INFO("Fitness Score: %f", icp.getFitnessScore());
			transformation_2 = icp.getFinalTransformation();
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_2 (0,0), transformation_2 (0,1), transformation_2 (0,2));
			pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_2 (1,0), transformation_2 (1,1), transformation_2 (1,2));
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_2 (2,0), transformation_2 (2,1), transformation_2 (2,2));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_2 (0,3), transformation_2 (1,3), transformation_2 (2,3));
			pcl::console::print_info ("\n");

			ROS_INFO("FINAL TRANSFORMATION");
			Eigen::Matrix4f transformation_f = transformation_1*transformation_2;
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_f (0,0), transformation_f (0,1), transformation_f (0,2));
			pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_f (1,0), transformation_f (1,1), transformation_f (1,2));
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_f (2,0), transformation_f (2,1), transformation_f (2,2));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_f (0,3), transformation_f (1,3), transformation_f (2,3));
			pcl::console::print_info ("\n");

			pcl::visualization::PCLVisualizer visu("Alignment");
			visu.addPointCloud (scene_temp, ColorHandlerT (scene_temp, 255.0, 255.0, 0.0), "scene_temp");
			visu.addPointCloud (scene, ColorHandlerT (scene, 255.0, 0.0, 0.0), "scene");
			visu.addPointCloud (obj_align, ColorHandlerT (obj_align, 0.0, 0.0, 255.0), "obj_align");
			visu.spin();
		}
		else
		{
			ROS_INFO("2. Alignment Fail!");
		}
	}
	return;
}
void BatteryPCLDetection::pclCallback(const sensor_msgs::PointCloud2ConstPtr& msg){ //only used for debugging
	ROS_INFO("####################");
	
	pcl::PCLPointCloud2* c = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr c_ptr(c);
	pcl::PCLPointCloud2 cloud_filtered;

	//Convert
	pcl_conversions::toPCL(*msg, *c);

	//Filter Cloud Voxel Grid
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(c_ptr);
	sor.setLeafSize(0.01, 0.01, 0.01);
	sor.filter(cloud_filtered);

	//Convert
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	//Publish
	pclPub.publish(output);
	return;
}

void BatteryPCLDetection::detect(int argc, char **argv) {

	n = ros::NodeHandle("~");		
	pListener = new (tf::TransformListener);
  	//pListener = new (tf::TransformListener);
	
  	posPub = n.advertise<geometry_msgs::PointStamped> ("/position_battery", 1);
	pclSub = n.subscribe("/camera/depth_registered/points", 1, &BatteryPCLDetection::pclAlignmentCallback, this);
	pclPub = n.advertise<sensor_msgs::PointCloud2>("/test_pcl_output", 1);

  	ros::Rate lr(10);

  	while(ros::ok())
  	{
      		ros::spinOnce();
      		lr.sleep();
  	}
}


int main (int argc, char **argv) {
	ros::init (argc, argv, "battery_detection");
  	BatteryPCLDetection detector = BatteryPCLDetection();
  	detector.detect(argc, argv);
}
