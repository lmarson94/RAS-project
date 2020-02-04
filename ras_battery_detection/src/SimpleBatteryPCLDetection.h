#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include <tf/transform_listener.h>
#include <vector>
#include <math.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Core>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


tf::TransformListener* pListener = NULL;

class SimpleBatteryPCLDetection
{
	public:
		SimpleBatteryPCLDetection();
		void detect (int argc, char **argv);
	private:
		ros::NodeHandle n;

		pcl::PCLPointCloud2* temp_cloud;
		ros::Subscriber pclSub;
		ros::Publisher posPub;
		ros::Publisher pclPub;
		void pclAlignmentCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void calcBatteryCenter(const pcl::PointCloud<PointNT>::Ptr sce, const pcl::PointCloud<PointNT>::Ptr bc, bool isVertical);
		bool basicBatteryCheck(const pcl::PointCloud<PointNT>::Ptr cloud);
		int callbackWait, currentCallbackIndex;

		// Point clouds
		PointCloudT::Ptr scene;
		ros::Time currentTimeStamp;

};
