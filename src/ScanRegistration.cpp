#include "ScanRegistration.h"

#include <pcl/filters/voxel_grid.h>

using namespace loam;

ScanRegistration::ScanRegistration()
{	
	initTime = 0;
	laserRotDir = 1;
}

ScanRegistration::~ScanRegistration()
{
	
}

void ScanRegistration::addScan(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn, double timestamp)
{	
	if (initTime == 0)
	{
		initTime = timestamp;
	}

	int cloudSize = laserCloudIn->points.size();
	
	// New temporary pc to hold some additional values
	// h: ?
	// s: Response of feature detector below
	// v: 2(sharp) / 1(less sharp) / -1(flat) / 0(less flat)
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>(cloudSize, 1));
	for (int i = 0; i < cloudSize; i++)
	{
		laserCloud->points[i].x = laserCloudIn->points[i].x;
		laserCloud->points[i].y = laserCloudIn->points[i].y;
		laserCloud->points[i].z = laserCloudIn->points[i].z;
		laserCloud->points[i].h = timestamp - initTime;
		laserCloud->points[i].v = 0;
		cloudSortInd[i] = i;
		cloudNeighborPicked[i] = 0;
	}

	// Feature point extraction (Section V-A)
	for (int i = 5; i < cloudSize - 5; i++)
	{
		float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
			+ laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
			+ laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
			+ laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
			+ laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
			+ laserCloud->points[i + 5].x;
		float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
			+ laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
			+ laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
			+ laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
			+ laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
			+ laserCloud->points[i + 5].y;
		float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
			+ laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
			+ laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
			+ laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
			+ laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
			+ laserCloud->points[i + 5].z;

		laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ; // c-Value
	}

	// Split scan in 4 regions
	int startPoints[4] = {	5,
								6 + int((cloudSize - 10) / 4.0),
								6 + int((cloudSize - 10) / 2.0),
								6 + int(3 * (cloudSize - 10) / 4.0)};

	int endPoints[4] = {	5 + int((cloudSize - 10) / 4.0),
							5 + int((cloudSize - 10) / 2.0),
							5 + int(3 * (cloudSize - 10) / 4.0),
							cloudSize - 6};

	for (int i = 0; i < 4; i++)
	{
		int sp = startPoints[i];
		int ep = endPoints[i];

		// Sort the points within each region based on their c-Value
		for (int j = sp + 1; j <= ep; j++)
		{
			for (int k = j; k >= sp + 1; k--)
			{
				if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s)
				{
					int temp = cloudSortInd[k - 1];
					cloudSortInd[k - 1] = cloudSortInd[k];
					cloudSortInd[k] = temp;
				}
			}
		}

		// Select the points with largest c-Value (edges)
		int largestPickedNum = 0;
		for (int j = ep; j >= sp; j--)
		{
			if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
			laserCloud->points[cloudSortInd[j]].s > 0.1 &&
			(fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
			fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
			fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
			fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
			fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
			fabs(laserCloud->points[cloudSortInd[j]].z) < 30)
			{

				largestPickedNum++;
				if (largestPickedNum <= 2)
				{
					laserCloud->points[cloudSortInd[j]].v = 2;
					mCurrentSweep.push_back(laserCloud->points[cloudSortInd[j]]);
				} else if (largestPickedNum <= 20)
				{
					laserCloud->points[cloudSortInd[j]].v = 1;
					mExtraPoints.push_back(laserCloud->points[cloudSortInd[j]]);
				}else
				{
					break;
				}
			}
		}

		// Select the points with smallest c-Value (surfaces)
		int smallestPickedNum = 0;
		for (int j = sp; j <= ep; j++)
		{
			if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
			laserCloud->points[cloudSortInd[j]].s < 0.1 &&
			(fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
			fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
			fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
			fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
			fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
			fabs(laserCloud->points[cloudSortInd[j]].z) < 30)
			{
				laserCloud->points[cloudSortInd[j]].v = -1;
				mCurrentSweep.push_back(laserCloud->points[cloudSortInd[j]]);

				smallestPickedNum++;
				if (smallestPickedNum >= 4)
				{
					break;
				}
			}
		}
	}

	// Select remaining points for mapping
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
	for (int i = 0; i < cloudSize; i++)
	{
		if (laserCloud->points[i].v == 0)
		{
			surfPointsLessFlat->push_back(laserCloud->points[i]);
		}
	}

	// Filter the remaining points by using only 1 point per 0.1m cube
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
	downSizeFilter.setInputCloud(surfPointsLessFlat);
	downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
	downSizeFilter.filter(*surfPointsLessFlatDS);

	mExtraPoints += *surfPointsLessFlatDS;
}

void ScanRegistration::finishSweep()
{
	mLastSweep = mCurrentSweep;
	mLastSweep += mExtraPoints;
	
	mCurrentSweep.clear();
	mExtraPoints.clear();
}

pcl::PointCloud<pcl::PointXYZHSV> ScanRegistration::getCurrentSweep()
{
	return mCurrentSweep;
}

pcl::PointCloud<pcl::PointXYZHSV> ScanRegistration::getLastSweep()
{
	return mLastSweep;
}