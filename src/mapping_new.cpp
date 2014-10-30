#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>


pcl::PointCloud<pcl::PointXYZ> gMap;
ros::Publisher mapPub;

tf::TransformListener* tfListener;

void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
	// Convert to PCL type
	pcl::PointCloud<pcl::PointXYZ> inputPCL;
	pcl::fromROSMsg(*cloudMsg, inputPCL);
	
	tf::StampedTransform odom_tf;
	Eigen::Affine3d odometry = Eigen::Affine3d::Identity();
	try
	{
		tfListener->lookupTransform("camera_init", "camera", cloudMsg->header.stamp, odom_tf);
		tf::transformTFToEigen(odom_tf, odometry);
	} catch(tf::TransformException e)
	{
		ROS_WARN("%s", e.what());
	}
	
	
	// Compute surface normals and curvature
	pcl::PointCloud<pcl::PointNormal> inputNomals;
	pcl::PointCloud<pcl::PointNormal> mapNomals;
	pcl::PointCloud<pcl::PointNormal> outputNormals;
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normEst;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	normEst.setSearchMethod (tree);
	normEst.setKSearch (30);
	
	normEst.setInputCloud (boost::make_shared< const pcl::PointCloud< pcl::PointXYZ > >(inputPCL));
	normEst.compute(inputNomals);
	pcl::copyPointCloud (inputPCL, inputNomals);

	normEst.setInputCloud (boost::make_shared< const pcl::PointCloud< pcl::PointXYZ > >(gMap));
	normEst.compute (mapNomals);
	pcl::copyPointCloud (gMap, mapNomals);
	
	// Align
	pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
	reg.setTransformationEpsilon (1e-6);
	
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (0.1);  
	reg.setInputSource(boost::make_shared< const pcl::PointCloud< pcl::PointNormal > >(inputNomals));
	reg.setInputTarget(boost::make_shared< const pcl::PointCloud< pcl::PointNormal > >(mapNomals));
	reg.setMaximumIterations (30);

	// Estimate
	reg.align(outputNormals);

	// Transform the source to target
	Eigen::Matrix4f transform = reg.getFinalTransformation();
	pcl::PointCloud<pcl::PointXYZ> alignedPCL;
	pcl::transformPointCloud(inputPCL, alignedPCL, transform);

	// Add to map
	gMap += alignedPCL;
	
	// Publish global map
	sensor_msgs::PointCloud2 map;
	pcl::toROSMsg(gMap, map);
	map.header.stamp = cloudMsg->header.stamp;
	map.header.frame_id = "/camera_init_2";
	mapPub.publish(map);	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "newMapping");
	ros::NodeHandle nh;
	
	tfListener = new tf::TransformListener(nh);
	
	ros::Subscriber pclSub = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 5, receivePointCloud);
	mapPub = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 1);
	
	ros::spin();
	return 0;
}