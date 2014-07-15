#include "ScanRegistration.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

loam::ScanRegistration* gScanRegistration;
ros::Publisher* gCurrentCloudPublisher;
ros::Publisher* gLastCloudPublisher;

double currentSweepStamp;
double lastSweepStamp;

unsigned int cycle;

using namespace ros;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
	// Transform into a pcl::PointCloud
	currentSweepStamp = laserCloudIn->header.stamp.toSec();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclIn(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*laserCloudIn, *pclIn);
	
	// Add new scan to current point cloud
	gScanRegistration->addScan(pclIn, laserCloudIn->header.stamp.toSec());
		
	if(cycle >= 3)
	{
		// Publish current accumulated point cloud
		sensor_msgs::PointCloud2 currentSweep;
		pcl::toROSMsg(gScanRegistration->getCurrentSweep(), currentSweep);
		currentSweep.header.stamp = laserCloudIn->header.stamp;
		currentSweep.header.frame_id = "/camera";
		gCurrentCloudPublisher->publish(currentSweep);

		// Publish the finished sweep as full point cloud
		sensor_msgs::PointCloud2 lastSweep;
		pcl::toROSMsg(gScanRegistration->getLastSweep(), lastSweep);
		lastSweep.header.stamp = ros::Time().fromSec(lastSweepStamp);
		lastSweep.header.frame_id = "/camera";
		gLastCloudPublisher->publish(lastSweep);
		cycle = 0;
	}
	cycle++;
}

void sweepHandler(const std_msgs::HeaderConstPtr& msg)
{
	// Finish the currently running sweep
	gScanRegistration->finishSweep();
	lastSweepStamp = currentSweepStamp;
}

int main(int argc, char **argv)
{
	init(argc, argv, "ScanRegistration");
	NodeHandle n;

	gScanRegistration = new loam::ScanRegistration();

	ros::Subscriber scanSubscriber = n.subscribe<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered", 2, laserCloudHandler);
	ros::Subscriber sweepSubscriber = n.subscribe<std_msgs::Header>("sweep_trigger", 5, sweepHandler);
	ros::Publisher currentCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("laser_cloud_extre_cur", 2);
	ros::Publisher lastCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("laser_cloud_last", 2);
	
	gCurrentCloudPublisher = &currentCloudPublisher;
	gLastCloudPublisher = &lastCloudPublisher;
	
	spin();
	
	delete gScanRegistration;
	return 0;	
}