#include "ScanRegistration.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

loam::ScanRegistration* gScanRegistration;
ros::Publisher* gCurrentCloudPublisher;
ros::Publisher* gLastCloudPublisher;
ros::Publisher* gScanPublisher;

double currentSweepStamp;
double lastSweepStamp;

int cycle;
int skip;

using namespace ros;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
	// Transform into a pcl::PointCloud
	currentSweepStamp = laserCloudIn->header.stamp.toSec();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclIn(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*laserCloudIn, *pclIn);
	
	// Switch coordinates according to LOAM
	unsigned int pclSize = pclIn->size();
	pcl::PointXYZ p;
	for(unsigned int i; i < pclSize; i++)
	{
		p = pclIn->points[i];
		pclIn->points[i].x = p.z;
		pclIn->points[i].y = -p.x;
		pclIn->points[i].z = -p.y;
	}
	
	// Republish the transformed cloud
	sensor_msgs::PointCloud2 fixedScan;
    pcl::toROSMsg(*pclIn, fixedScan);
    fixedScan.header.stamp = laserCloudIn->header.stamp;
    fixedScan.header.frame_id = "/camera";
    gScanPublisher->publish(fixedScan);
	
	// Add new scan to current point cloud
	gScanRegistration->addScan(pclIn, laserCloudIn->header.stamp.toSec());
		
	if(cycle >= skip)
	{
		// Publish current accumulated point cloud
		sensor_msgs::PointCloud2 currentSweep;
		pcl::toROSMsg(gScanRegistration->getCurrentSweep(), currentSweep);
		currentSweep.header.stamp = laserCloudIn->header.stamp;
		currentSweep.header.frame_id = "/camera";
		gCurrentCloudPublisher->publish(currentSweep);

		ROS_DEBUG("Running sweep for odometry: %d points", currentSweep.width);
		cycle = 0;
	}
	cycle++;
}

void sweepHandler(const std_msgs::HeaderConstPtr& msg)
{
	// Finish the currently running sweep
	gScanRegistration->finishSweep();
	lastSweepStamp = currentSweepStamp;
	
	// Publish the finished sweep as full point cloud
	sensor_msgs::PointCloud2 lastSweep;
	pcl::toROSMsg(gScanRegistration->getLastSweep(), lastSweep);
	lastSweep.header.stamp = ros::Time().fromSec(lastSweepStamp);
	lastSweep.header.frame_id = "/camera";
	gLastCloudPublisher->publish(lastSweep);
	
	ROS_DEBUG("Full sweep for mapping: %d points.", lastSweep.width);
}

int main(int argc, char **argv)
{
	init(argc, argv, "ScanRegistration");
	NodeHandle n;
	n.param("frame_skip", skip, 3);
	if(skip < 1)
	{
		skip = 3;
		ROS_WARN("Parameter 'frame_skip' must be >= 1, using default of 3.");
	}

	gScanRegistration = new loam::ScanRegistration();

	ros::Subscriber scanSubscriber = n.subscribe<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered", 2, laserCloudHandler);
	ros::Subscriber sweepSubscriber = n.subscribe<std_msgs::Header>("sweep_trigger", 5, sweepHandler);
	ros::Publisher currentCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("laser_cloud_extre_cur", 2);
	ros::Publisher lastCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("laser_cloud_last", 2);
	ros::Publisher scanPublisher = n.advertise<sensor_msgs::PointCloud2>("fixed_scan", 2);
	
	gCurrentCloudPublisher = &currentCloudPublisher;
	gLastCloudPublisher = &lastCloudPublisher;
	gScanPublisher = &scanPublisher;
	
	spin();
	
	delete gScanRegistration;
	return 0;	
}