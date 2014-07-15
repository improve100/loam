#include "ScanRegistration.h"

#include <pcl/filters/voxel_grid.h>

using namespace loam;

ScanRegistration::ScanRegistration()
{	
	initTime = 0;
	timeStart = 0;
	timeLasted = 0;
	
	timeScanCur = 0;
	timeScanLast = 0;	
	
	laserRotDir = 1;
	laserAngleLast = 0;
	laserAngleCur = 0;
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

	// Do some increadible magic:
	timeScanLast = timeScanCur;
	timeScanCur = timestamp;
	timeLasted = timeScanCur - initTime;	
	
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>(cloudSize, 1));
	for (int i = 0; i < cloudSize; i++)
	{
		laserCloud->points[i].x = laserCloudIn->points[i].x;
		laserCloud->points[i].y = laserCloudIn->points[i].y;
		laserCloud->points[i].z = laserCloudIn->points[i].z;
		laserCloud->points[i].h = timeLasted;
		laserCloud->points[i].v = 0;
		cloudSortInd[i] = i;
		cloudNeighborPicked[i] = 0;
	}

	// What is happening here?
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

		laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
	}

	for (int i = 5; i < cloudSize - 6; i++)
	{
		float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
		float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
		float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
		float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		if (diff > 0.05)
		{
			float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
			laserCloud->points[i].y * laserCloud->points[i].y +
			laserCloud->points[i].z * laserCloud->points[i].z);

			float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
			laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
			laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

			if (depth1 > depth2)
			{
				diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
				diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
				diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1)
				{
					cloudNeighborPicked[i - 5] = 1;
					cloudNeighborPicked[i - 4] = 1;
					cloudNeighborPicked[i - 3] = 1;
					cloudNeighborPicked[i - 2] = 1;
					cloudNeighborPicked[i - 1] = 1;
					cloudNeighborPicked[i] = 1;
				}
			}else
			{
				diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
				diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
				diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
				{
					cloudNeighborPicked[i + 1] = 1;
					cloudNeighborPicked[i + 2] = 1;
					cloudNeighborPicked[i + 3] = 1;
					cloudNeighborPicked[i + 4] = 1;
					cloudNeighborPicked[i + 5] = 1;
					cloudNeighborPicked[i + 6] = 1;
				}
			}
		}

		float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
		float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
		float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

		float dis = laserCloud->points[i].x * laserCloud->points[i].x
			+ laserCloud->points[i].y * laserCloud->points[i].y
			+ laserCloud->points[i].z * laserCloud->points[i].z;

		if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis)
		{
			cloudNeighborPicked[i] = 1;
		}
	}
	
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

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
					cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
				} else if (largestPickedNum <= 20)
				{
					laserCloud->points[cloudSortInd[j]].v = 1;
					cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
				}else
				{
					break;
				}

				cloudNeighborPicked[cloudSortInd[j]] = 1;
				for (int k = 1; k <= 5; k++)
				{
					float diffX = laserCloud->points[cloudSortInd[j] + k].x - laserCloud->points[cloudSortInd[j] + k - 1].x;
					float diffY = laserCloud->points[cloudSortInd[j] + k].y - laserCloud->points[cloudSortInd[j] + k - 1].y;
					float diffZ = laserCloud->points[cloudSortInd[j] + k].z - laserCloud->points[cloudSortInd[j] + k - 1].z;
					if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
					{
						break;
					}
					cloudNeighborPicked[cloudSortInd[j] + k] = 1;
				}

				for (int k = -1; k >= -5; k--)
				{
					float diffX = laserCloud->points[cloudSortInd[j] + k].x - laserCloud->points[cloudSortInd[j] + k + 1].x;
					float diffY = laserCloud->points[cloudSortInd[j] + k].y - laserCloud->points[cloudSortInd[j] + k + 1].y;
					float diffZ = laserCloud->points[cloudSortInd[j] + k].z - laserCloud->points[cloudSortInd[j] + k + 1].z;
					if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
					{
						break;
					}
					cloudNeighborPicked[cloudSortInd[j] + k] = 1;
				}
			}
		}

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
				surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

				smallestPickedNum++;
				if (smallestPickedNum >= 4)
				{
					break;
				}

				cloudNeighborPicked[cloudSortInd[j]] = 1;
				for (int k = 1; k <= 5; k++)
				{
					float diffX = laserCloud->points[cloudSortInd[j] + k].x - laserCloud->points[cloudSortInd[j] + k - 1].x;
					float diffY = laserCloud->points[cloudSortInd[j] + k].y - laserCloud->points[cloudSortInd[j] + k - 1].y;
					float diffZ = laserCloud->points[cloudSortInd[j] + k].z - laserCloud->points[cloudSortInd[j] + k - 1].z;
					if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
					{
						break;
					}

					cloudNeighborPicked[cloudSortInd[j] + k] = 1;
				}
				for (int k = -1; k >= -5; k--)
				{
					float diffX = laserCloud->points[cloudSortInd[j] + k].x - laserCloud->points[cloudSortInd[j] + k + 1].x;
					float diffY = laserCloud->points[cloudSortInd[j] + k].y - laserCloud->points[cloudSortInd[j] + k + 1].y;
					float diffZ = laserCloud->points[cloudSortInd[j] + k].z - laserCloud->points[cloudSortInd[j] + k + 1].z;
					if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
					{
						break;
					}
					cloudNeighborPicked[cloudSortInd[j] + k] = 1;
				}
			}
		}
	}

	for (int i = 0; i < cloudSize; i++)
	{
		if (laserCloud->points[i].v == 0)
		{
			surfPointsLessFlat->push_back(laserCloud->points[i]);
		}
	}

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
	downSizeFilter.setInputCloud(surfPointsLessFlat);
	downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
	downSizeFilter.filter(*surfPointsLessFlatDS);

	mCurrentSweep += *cornerPointsSharp;
	mCurrentSweep += *surfPointsFlat;
	laserCloudLessExtreCur += *cornerPointsLessSharp;
	laserCloudLessExtreCur += *surfPointsLessFlatDS;

	laserCloudIn->clear();
	laserCloud->clear();
	cornerPointsSharp->clear();
	cornerPointsLessSharp->clear();
	surfPointsFlat->clear();
	surfPointsLessFlat->clear();
	surfPointsLessFlatDS->clear();

}

void ScanRegistration::finishSweep()
{
	timeStart = timeScanLast - initTime;
	
	mCurrentSweep += laserCloudLessExtreCur;
	mLastSweep = mCurrentSweep;
	mCurrentSweep.clear();
	
    laserCloudLessExtreCur.clear();
}

pcl::PointCloud<pcl::PointXYZHSV> ScanRegistration::getCurrentSweep()
{
	return mCurrentSweep;
}

pcl::PointCloud<pcl::PointXYZHSV> ScanRegistration::getLastSweep()
{
	return mLastSweep;
}