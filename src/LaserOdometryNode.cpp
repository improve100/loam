#include "LaserOdometry.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace loam;
using namespace ros;
using namespace tf;

LaserOdometry* gLaserOdometry;
TransformListener* tfListener;
TransformBroadcaster* tfBroadcaster;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr gCurrentSweep;

void currentSweepHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::fromROSMsg(*laserCloudIn, *laserCloud);

	gLaserOdometry->addCurrentSweep(laserCloud, laserCloudIn->header.stamp.toSec());
	
	// Translation and Rotation of the calculated Odometry
	double tx = 0;
	double ty = 0;
	double tz = 0;
	double rx = 0;
	double ry = 0;
	double rz = 0;
	
	// Publish the calculated odometry via TF
	StampedTransform laserOdometry;
	laserOdometry.setOrigin(Vector3(tx, ty, tz));
	laserOdometry.setRotation(createQuaternionFromRPY(rz, -rx, -ry));
	laserOdometry.frame_id_ = "/camera_init";
	laserOdometry.child_frame_id_ = "/camera";
	laserOdometry.stamp_ = laserCloudIn->header.stamp;
	tfBroadcaster->sendTransform(laserOdometry);
}

void lastSweepHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{

}

int main(int argc, char **argv)
{
	init(argc, argv, "LaserOdometry");
	NodeHandle n;
	
	Subscriber s1 = n.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_extre_cur", 2, currentSweepHandler);
	Subscriber s2 = n.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_last", 2, lastSweepHandler);
	
	tfListener = new tf::TransformListener(n);
	tfBroadcaster = new tf::TransformBroadcaster();
	
	gLaserOdometry = new LaserOdometry();
	
	spin();
	
	// Cleanup
	delete gLaserOdometry;
	delete tfListener;
	delete tfBroadcaster;
}