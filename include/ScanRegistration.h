#ifndef LOAM_SCANREGISTRATION_H
#define LOAM_SCANREGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace loam
{
	class ScanRegistration
	{
	public:
		ScanRegistration();
		~ScanRegistration();

		void addScan(pcl::PointCloud<pcl::PointXYZ>::Ptr scan, double timestamp);
		void finishSweep();
		pcl::PointCloud<pcl::PointXYZHSV> getCurrentSweep();
		pcl::PointCloud<pcl::PointXYZHSV> getLastSweep();

	private:
		pcl::PointCloud<pcl::PointXYZHSV> mCurrentSweep;
		pcl::PointCloud<pcl::PointXYZHSV> mLastSweep;
		pcl::PointCloud<pcl::PointXYZHSV> mExtraPoints;
		
		// To be removed / renamed
		double initTime;
		double timeStart;
		double timeLasted;
		
		double timeScanCur;
		double timeScanLast;
		
		int cloudSortInd[800];
		int cloudNeighborPicked[800];
		
		int laserRotDir;
		float laserAngleLast;
		float laserAngleCur;
	};
}

#endif