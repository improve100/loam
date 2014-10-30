#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

ros::Subscriber gCloudSubscriber;
ros::Publisher gCloudPublisher;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
// Switch coordinates according to LOAM
	unsigned int pclSize = pclIn->size();
	pcl::PointXYZHSV p;
	for(unsigned int i; i < pclSize; i++)
	{
		p = pclIn->points[i];
		pclIn->points[i].x = p.z;
		pclIn->points[i].y = -p.x;
		pclIn->points[i].z = -p.;
	}
}	

int main(int argc, char **argv)
{
	init(argc, argv, "pcl_transformer");
	NodeHandle n;
	
	gCloudSubscriber = n.subscribe<sensor_msgs::PointCloud2>("scan_in", 2, laserCloudHandler);
	gCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("scan_out", 2);

	spin();
	return 0;
}