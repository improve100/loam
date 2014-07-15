#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace ros;

int laserRotDir = 1;
float laserAngleLast = 0;
float laserAngleCur = 0;

Publisher* gSweepPublisher;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*laserCloudIn, *laserCloud);
	
	int cloudSize = laserCloud->points.size();
	
	bool newSweep = false;
	laserAngleLast = laserAngleCur;
	laserAngleCur = atan2(	laserCloud->points[cloudSize - 1].y - laserCloud->points[0].y, 
								laserCloud->points[cloudSize - 1].x - laserCloud->points[0].x);

	if (laserAngleLast > 0 && laserRotDir == 1 && laserAngleCur < laserAngleLast)
	{
		laserRotDir = -1;
		newSweep = true;
	} else if (laserAngleLast < 0 && laserRotDir == -1 && laserAngleCur > laserAngleLast)
	{
		laserRotDir = 1;
		newSweep = true;
	}
	
	if(newSweep)
	{
		std_msgs::Header head;
		head.stamp == Time::now();
		gSweepPublisher->publish(head);		
	}
}

int main(int argc, char **argv)
{
	init(argc, argv, "Trigger");
	NodeHandle n;
	
	Subscriber scanSubscriber = n.subscribe<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered", 2, laserCloudHandler);
	Publisher sweepPublisher = n.advertise<std_msgs::Header>("sweep_trigger", 5);
	gSweepPublisher = &sweepPublisher;
	
	spin();
	return 0;
}