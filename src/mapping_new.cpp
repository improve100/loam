#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
	// Convert to PCL type
	pcl::PointCloud<pcl::PointXYZ> inputPCL;
	pcl::fromROSMsg(*cloudMsg, inputPCL);
	
	// 
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "newMapping");
	ros::NodeHandle nh;
	
	ros::Subscriber pclSub = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 5, receivePointCloud);
	
	ros::spin();
	return 0;
}