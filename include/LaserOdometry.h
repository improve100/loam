#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace loam
{
	class LaserOdometry
	{
	public:
		LaserOdometry();
		~LaserOdometry();
		
		void addCurrentSweep(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudIn, double timestamp);
		void addLastSweep(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudIn, double timestamp);

	private:
		void transformReset();
		void transformToStart(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime);
		void transformToEnd(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime);
	
	public:
		// To remove / rename
		float transform[6];
		float transformRec[6];
		float transformSum[6];
		
		double initTime;
		double timeLasted;
		double timeLastedRec;
		double startTimeCur;
		double startTimeLast;
		
		pcl::PointCloud<pcl::PointXYZHSV> laserCloudExtreLast;
		pcl::PointCloud<pcl::PointXYZHSV> laserCloudSurfLast;
		pcl::PointCloud<pcl::PointXYZHSV> laserCloudCornerLast;
	
	};
}

#endif