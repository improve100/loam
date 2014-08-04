#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#define MAGIC_PARAM 1.0

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

bool systemInited = false;

double initTime;
double timeLasted;
double timeLastedRec;
double startTimeCur;
double startTimeLast;

double timeLaserCloudExtreCur = 0;
double timeLaserCloudLast = 0;

bool newLaserCloudLast = false;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreOri(new pcl::PointCloud<pcl::PointXYZHSV>());

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLLast(new pcl::PointCloud<pcl::PointXYZHSV>());

pcl::PointCloud<pcl::PointXYZHSV>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());

float transform[6] = {0};
float transformRec[6] = {0};
float transformSum[6] = {0};

tf::Transform tfTransform;
tf::Transform tfTransformSum;
tf::Transform tfTransformRec;

ros::Publisher pubLaserCloudLast2;
tf::TransformBroadcaster* tfBroadcaster;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;
std::vector<int> pointSelInd;

pcl::PointXYZHSV extreOri, extreSel, extreProj, tripod1, tripod2, tripod3, coeff;

void TransformToStart(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
{
  float s = (pi->h - startTime) / (endTime - startTime);

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->h = pi->h;
  po->s = pi->s;
  po->v = pi->v;
}

void TransformToEnd(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
{
  float s = (pi->h - startTime) / (endTime - startTime);

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transform[0];
  ry = transform[1];
  rz = transform[2];
  tx = transform[3];
  ty = transform[4];
  tz = transform[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  float x7 = cos(0) * (x6 - 0) 
           - sin(0) * (y6 - 0);
  float y7 = sin(0) * (x6 - 0) 
           + cos(0) * (y6 - 0);
  float z7 = z6 - 0;

  float x8 = x7;
  float y8 = cos(0) * y7 - sin(0) * z7;
  float z8 = sin(0) * y7 + cos(0) * z7;

  float x9 = cos(0) * x8 + sin(0) * z8;
  float y9 = y8;
  float z9 = -sin(0) * x8 + cos(0) * z8;

  float x10 = cos(0) * x9 - sin(0) * z9;
  float y10 = y9;
  float z10 = sin(0) * x9 + cos(0) * z9;

  float x11 = x10;
  float y11 = cos(0) * y10 + sin(0) * z10;
  float z11 = -sin(0) * y10 + cos(0) * z10;

  po->x = cos(0) * x11 + sin(0) * y11;
  po->y = -sin(0) * x11 + cos(0) * y11;
  po->z = z11;
  po->h = pi->h;
  po->s = pi->s;
  po->v = pi->v;
}

void AccumulateRotation()
{
	float cx = transformSum[0];
	float cy = transformSum[1];
	float cz = transformSum[2];
	
	float lx = -transform[0];
	float ly = -transform[1] * MAGIC_PARAM;
	float lz = -transform[2];
	
	float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
	float rx = -asin(srx);

	float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
			   + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
	float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
			   - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
	float ry = atan2(srycrx / cos(rx), crycrx / cos(rx));

	float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
			   + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
	float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
			   - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
	float rz = atan2(srzcrx / cos(rx), crzcrx / cos(rx));

    float x1 = cos(rz) * (transform[3]) 
			 - sin(rz) * (transform[4]);
	float y1 = sin(rz) * (transform[3]) 
			 + cos(rz) * (transform[4]);
	float z1 = transform[5] * MAGIC_PARAM;

	float x2 = x1;
	float y2 = cos(rx) * y1 - sin(rx) * z1;
	float z2 = sin(rx) * y1 + cos(rx) * z1;

	transformSum[0] = rx;
	transformSum[1] = ry;
	transformSum[2] = rz;
	transformSum[3] = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
	transformSum[4] = transformSum[4] - y2;
	transformSum[5] = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);
}

void laserCloudExtreCurHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudExtreCur2)
{
	if (!systemInited)
	{
		initTime = laserCloudExtreCur2->header.stamp.toSec();
		systemInited = true;
	}
	timeLaserCloudExtreCur = laserCloudExtreCur2->header.stamp.toSec();
	timeLasted = timeLaserCloudExtreCur - initTime;

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur3(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::fromROSMsg(*laserCloudExtreCur2, *laserCloudExtreCur3);
	int laserCloudExtreCur3Size = laserCloudExtreCur3->points.size();

	laserCloudExtreCur->clear();
	for (int i = 0; i < laserCloudExtreCur3Size; i++)
	{
		laserCloudExtreCur->push_back(laserCloudExtreCur3->points[i]);
	}
	laserCloudExtreCur3->clear();

	if (timeLasted <= 4.0)
		return;
	
	// Everything from main()
	double startTime, endTime;

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr extrePointsPtr, laserCloudCornerPtr, laserCloudSurfPtr;
	pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerPtr, kdtreeSurfPtr;

	if(newLaserCloudLast)
	{
		startTime = startTimeLast;
		endTime = startTimeCur;

		extrePointsPtr = laserCloudExtreLast;
		laserCloudCornerPtr = laserCloudCornerLLast;
		laserCloudSurfPtr = laserCloudSurfLLast;
		kdtreeCornerPtr = kdtreeCornerLLast;
		kdtreeSurfPtr = kdtreeSurfLLast;
	}else
	{
		startTime = startTimeCur;
		endTime = timeLasted;

		extrePointsPtr = laserCloudExtreCur;
		laserCloudCornerPtr = laserCloudCornerLast;
		laserCloudSurfPtr = laserCloudSurfLast;
		kdtreeCornerPtr = kdtreeCornerLast;
		kdtreeSurfPtr = kdtreeSurfLast;
	}

	// Do something with the odometry estimation
	float s = (timeLasted - timeLastedRec) / (startTimeCur - startTimeLast);
	for (int i = 0; i < 6; i++)
	{
		transform[i] += s * transformRec[i];        
	}
	timeLastedRec = timeLasted;

	if (laserCloudSurfPtr->points.size() >= 100)
	{
		newLaserCloudLast = false;

		int extrePointNum = extrePointsPtr->points.size();
		int laserCloudCornerNum = laserCloudCornerPtr->points.size();
		int laserCloudSurfNum = laserCloudSurfPtr->points.size();

		float st = (timeLasted - startTime) / (startTimeCur - startTimeLast);
		int iterNum = st * 50;

		int pointSelSkipNum = 2;
		for (int iterCount = 0; iterCount < iterNum; iterCount++)
		{
			laserCloudExtreOri->clear();
			coeffSel->clear();

			bool isPointSel = false;
			if (iterCount % (pointSelSkipNum + 1) == 0)
			{
				isPointSel = true;
				pointSelInd.clear();
			}

			for (int i = 0; i < extrePointNum; i++)
			{
				extreOri = extrePointsPtr->points[i];
				TransformToStart(&extreOri, &extreSel, startTime, endTime);

				if (isPointSel)
				{
					pointSelInd.push_back(-1);
					pointSelInd.push_back(-1);
					pointSelInd.push_back(-1);
				}

				if (fabs(extreOri.v + 1) < 0.05)
				{
					int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
					if (isPointSel)
					{
						kdtreeSurfPtr->nearestKSearch(extreSel, 1, pointSearchInd, pointSearchSqDis);
						if (pointSearchSqDis[0] > 1.0)
						{
							continue;
						}

						closestPointInd = pointSearchInd[0];
						float closestPointTime = laserCloudSurfPtr->points[closestPointInd].h;

						float pointSqDis, minPointSqDis2 = 1, minPointSqDis3 = 1;
						for (int j = closestPointInd + 1; j < laserCloudSurfNum; j++)
						{
							if (laserCloudSurfPtr->points[j].h > closestPointTime + 0.07)
							{
								break;
							}

							pointSqDis = (laserCloudSurfPtr->points[j].x - extreSel.x) * 
							(laserCloudSurfPtr->points[j].x - extreSel.x) + 
							(laserCloudSurfPtr->points[j].y - extreSel.y) * 
							(laserCloudSurfPtr->points[j].y - extreSel.y) + 
							(laserCloudSurfPtr->points[j].z - extreSel.z) * 
							(laserCloudSurfPtr->points[j].z - extreSel.z);

							if (laserCloudSurfPtr->points[j].h < closestPointTime + 0.005)
							{
								if (pointSqDis < minPointSqDis2) {
								minPointSqDis2 = pointSqDis;
								minPointInd2 = j;
								}
							} else
							{
								if (pointSqDis < minPointSqDis3)
								{	// Welcome to the 10th indentation level!
									minPointSqDis3 = pointSqDis;
									minPointInd3 = j;
								}
							}
						}
						for (int j = closestPointInd - 1; j >= 0; j--)
						{
							if (laserCloudSurfPtr->points[j].h < closestPointTime - 0.07)
							{
								break;
							}

							pointSqDis = (laserCloudSurfPtr->points[j].x - extreSel.x) * 
							(laserCloudSurfPtr->points[j].x - extreSel.x) + 
							(laserCloudSurfPtr->points[j].y - extreSel.y) * 
							(laserCloudSurfPtr->points[j].y - extreSel.y) + 
							(laserCloudSurfPtr->points[j].z - extreSel.z) * 
							(laserCloudSurfPtr->points[j].z - extreSel.z);

							if (laserCloudSurfPtr->points[j].h > closestPointTime - 0.005)
							{
								if (pointSqDis < minPointSqDis2)
								{
									minPointSqDis2 = pointSqDis;
									minPointInd2 = j;
								}
							} else
							{
								if (pointSqDis < minPointSqDis3)
								{
									minPointSqDis3 = pointSqDis;
									minPointInd3 = j;
								}
							}
						}
					} else
					{
						if (pointSelInd[3 * i] >= 0)
						{
							closestPointInd = pointSelInd[3 * i];
							minPointInd2 = pointSelInd[3 * i + 1];
							minPointInd3 = pointSelInd[3 * i + 2];

							float distX = extreSel.x - laserCloudSurfPtr->points[closestPointInd].x;
							float distY = extreSel.y - laserCloudSurfPtr->points[closestPointInd].y;
							float distZ = extreSel.z - laserCloudSurfPtr->points[closestPointInd].z;
							if (distX * distX + distY * distY + distZ * distZ > 1.0)
							{
								continue;
							}
						} else
						{
							continue;
						}
					}

					if (minPointInd2 >= 0 && minPointInd3 >= 0)
					{
						tripod1 = laserCloudSurfPtr->points[closestPointInd];
						tripod2 = laserCloudSurfPtr->points[minPointInd2];
						tripod3 = laserCloudSurfPtr->points[minPointInd3];

						float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
						- (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
						float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
						- (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
						float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
						- (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
						float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

						float ps = sqrt(pa * pa + pb * pb + pc * pc);
						pa /= ps;
						pb /= ps;
						pc /= ps;
						pd /= ps;

						float pd2 = pa * extreSel.x + pb * extreSel.y + pc * extreSel.z + pd;

						extreProj = extreSel;
						extreProj.x -= pa * pd2;
						extreProj.y -= pb * pd2;
						extreProj.z -= pc * pd2;

						float s = 1;
						if (iterCount >= 30)
						{
							s = 1 - 8 * fabs(pd2) / sqrt(sqrt(extreSel.x * extreSel.x
							+ extreSel.y * extreSel.y + extreSel.z * extreSel.z));
						}

						coeff.x = s * pa;
						coeff.y = s * pb;
						coeff.z = s * pc;
						coeff.h = s * pd2;

						if (s > 0.2 || iterNum < 30)
						{
							laserCloudExtreOri->push_back(extreOri);
							coeffSel->push_back(coeff);

							if (isPointSel)
							{
								pointSelInd[3 * i] = closestPointInd;
								pointSelInd[3 * i + 1] = minPointInd2;
								pointSelInd[3 * i + 2] = minPointInd3;
							}
						}
					}
				} else if (fabs(extreOri.v - 2) < 0.05)
				{
					int closestPointInd = -1, minPointInd2 = -1;
					if (isPointSel)
					{
						kdtreeCornerPtr->nearestKSearch(extreSel, 1, pointSearchInd, pointSearchSqDis);
						if (pointSearchSqDis[0] > 1.0)
						{
							continue;
						}

						closestPointInd = pointSearchInd[0];
						float closestPointTime = laserCloudCornerPtr->points[closestPointInd].h;

						float pointSqDis, minPointSqDis2 = 1;
						for (int j = closestPointInd + 1; j < laserCloudCornerNum; j++)
						{
							if (laserCloudCornerPtr->points[j].h > closestPointTime + 0.07)
							{
								break;
							}

							pointSqDis = (laserCloudCornerPtr->points[j].x - extreSel.x) * 
							(laserCloudCornerPtr->points[j].x - extreSel.x) + 
							(laserCloudCornerPtr->points[j].y - extreSel.y) * 
							(laserCloudCornerPtr->points[j].y - extreSel.y) + 
							(laserCloudCornerPtr->points[j].z - extreSel.z) * 
							(laserCloudCornerPtr->points[j].z - extreSel.z);

							if (laserCloudCornerPtr->points[j].h > closestPointTime + 0.005)
							{
								if (pointSqDis < minPointSqDis2)
								{
									minPointSqDis2 = pointSqDis;
									minPointInd2 = j;
								}
							}
						}
						for (int j = closestPointInd - 1; j >= 0; j--)
						{
							if (laserCloudCornerPtr->points[j].h < closestPointTime - 0.07)
							{
								break;
							}

							pointSqDis = (laserCloudCornerPtr->points[j].x - extreSel.x) * 
							(laserCloudCornerPtr->points[j].x - extreSel.x) + 
							(laserCloudCornerPtr->points[j].y - extreSel.y) * 
							(laserCloudCornerPtr->points[j].y - extreSel.y) + 
							(laserCloudCornerPtr->points[j].z - extreSel.z) * 
							(laserCloudCornerPtr->points[j].z - extreSel.z);

							if (laserCloudCornerPtr->points[j].h < closestPointTime - 0.005)
							{
								if (pointSqDis < minPointSqDis2)
								{
									minPointSqDis2 = pointSqDis;
									minPointInd2 = j;
								}
							}
						}
					} else
					{
						if (pointSelInd[3 * i] >= 0)
						{
							closestPointInd = pointSelInd[3 * i];
							minPointInd2 = pointSelInd[3 * i + 1];

							float distX = extreSel.x - laserCloudCornerPtr->points[closestPointInd].x;
							float distY = extreSel.y - laserCloudCornerPtr->points[closestPointInd].y;
							float distZ = extreSel.z - laserCloudCornerPtr->points[closestPointInd].z;
							if (distX * distX + distY * distY + distZ * distZ > 1.0)
							{
								continue;
							}
						} else
						{
							continue;
						}
					}

					if (minPointInd2 >= 0)
					{
						tripod1 = laserCloudCornerPtr->points[closestPointInd];
						tripod2 = laserCloudCornerPtr->points[minPointInd2];

						float x0 = extreSel.x;
						float y0 = extreSel.y;
						float z0 = extreSel.z;
						float x1 = tripod1.x;
						float y1 = tripod1.y;
						float z1 = tripod1.z;
						float x2 = tripod2.x;
						float y2 = tripod2.y;
						float z2 = tripod2.z;

						float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
						+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
						* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
						+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
						* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

						float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

						float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
						+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

						float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
						- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

						float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
						+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

						float ld2 = a012 / l12;

						extreProj = extreSel;
						extreProj.x -= la * ld2;
						extreProj.y -= lb * ld2;
						extreProj.z -= lc * ld2;

						float s = 2 * (1 - 8 * fabs(ld2));

						coeff.x = s * la;
						coeff.y = s * lb;
						coeff.z = s * lc;
						coeff.h = s * ld2;

						if (s > 0.4)
						{
							laserCloudExtreOri->push_back(extreOri);
							coeffSel->push_back(coeff);

							if (isPointSel)
							{
								pointSelInd[3 * i] = closestPointInd;
								pointSelInd[3 * i + 1] = minPointInd2;
							}
						}
					}
				}
			}
			int extrePointSelNum = laserCloudExtreOri->points.size();

			if (extrePointSelNum < 10)
			{
				continue;
			}

			cv::Mat matA(extrePointSelNum, 6, CV_32F, cv::Scalar::all(0));
			cv::Mat matAt(6, extrePointSelNum, CV_32F, cv::Scalar::all(0));
			cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
			cv::Mat matB(extrePointSelNum, 1, CV_32F, cv::Scalar::all(0));
			cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
			cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
			
			for (int i = 0; i < extrePointSelNum; i++)
			{
				extreOri = laserCloudExtreOri->points[i];
				coeff = coeffSel->points[i];

				float s = (extreOri.h - startTime) / (endTime - startTime);

				float srx = sin(s * transform[0]);
				float crx = cos(s * transform[0]);
				float sry = sin(s * transform[1]);
				float cry = cos(s * transform[1]);
				float srz = sin(s * transform[2]);
				float crz = cos(s * transform[2]);
				float tx = s * transform[3];
				float ty = s * transform[4];
				float tz = s * transform[5];

				float arx = (-s*crx*sry*srz*extreOri.x + s*crx*crz*sry*extreOri.y + s*srx*sry*extreOri.z 
				+ s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
				+ (s*srx*srz*extreOri.x - s*crz*srx*extreOri.y + s*crx*extreOri.z
				+ s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
				+ (s*crx*cry*srz*extreOri.x - s*crx*cry*crz*extreOri.y - s*cry*srx*extreOri.z
				+ s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

				float ary = ((-s*crz*sry - s*cry*srx*srz)*extreOri.x 
				+ (s*cry*crz*srx - s*sry*srz)*extreOri.y - s*crx*cry*extreOri.z 
				+ tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx) 
				+ s*tz*crx*cry) * coeff.x
				+ ((s*cry*crz - s*srx*sry*srz)*extreOri.x 
				+ (s*cry*srz + s*crz*srx*sry)*extreOri.y - s*crx*sry*extreOri.z
				+ s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry) 
				- tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

				float arz = ((-s*cry*srz - s*crz*srx*sry)*extreOri.x + (s*cry*crz - s*srx*sry*srz)*extreOri.y
				+ tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
				+ (-s*crx*crz*extreOri.x - s*crx*srz*extreOri.y
				+ s*ty*crx*srz + s*tx*crx*crz) * coeff.y
				+ ((s*cry*crz*srx - s*sry*srz)*extreOri.x + (s*crz*sry + s*cry*srx*srz)*extreOri.y
				+ tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

				float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y 
				- s*(crz*sry + cry*srx*srz) * coeff.z;

				float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y 
				- s*(sry*srz - cry*crz*srx) * coeff.z;

				float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;

				float d2 = coeff.h;

				matA.at<float>(i, 0) = arx;
				matA.at<float>(i, 1) = ary;
				matA.at<float>(i, 2) = arz;
				matA.at<float>(i, 3) = atx;
				matA.at<float>(i, 4) = aty;
				matA.at<float>(i, 5) = atz;
				matB.at<float>(i, 0) = -0.015 * st * d2;
			}
			cv::transpose(matA, matAt);
			matAtA = matAt * matA; //+ 0.1 * cv::Mat::eye(6, 6, CV_32F);
			matAtB = matAt * matB;
			cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

			if (fabs(matX.at<float>(0, 0)) < 0.005 &&
			fabs(matX.at<float>(1, 0)) < 0.005 &&
			fabs(matX.at<float>(2, 0)) < 0.005 &&
			fabs(matX.at<float>(3, 0)) < 0.01 &&
			fabs(matX.at<float>(4, 0)) < 0.01 &&
			fabs(matX.at<float>(5, 0)) < 0.01)
			{
				transform[0] += 0.1 * matX.at<float>(0, 0);
				transform[1] += 0.1 * matX.at<float>(1, 0);
				transform[2] += 0.1 * matX.at<float>(2, 0);
				transform[3] += matX.at<float>(3, 0);
				transform[4] += matX.at<float>(4, 0);
				transform[5] += matX.at<float>(5, 0);
			} else
			{
				ROS_WARN ("Odometry update out of bound");
			}
		}
	}

	// Publish via tf	
	tfTransform.setRotation(tf::Quaternion(transform[0], transform[1], transform[2]));
	tfTransform.setOrigin(tf::Vector3(transform[3], transform[4], transform[5]));
	tfBroadcaster->sendTransform(tf::StampedTransform(tfTransform.inverse(), laserCloudExtreCur2->header.stamp, "transformSum", "camera"));
	
	tfTransformSum.setRotation(tf::Quaternion(transformSum[0], transformSum[1], transformSum[2]));
	tfTransformSum.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
	tfBroadcaster->sendTransform(tf::StampedTransform(tfTransformSum, laserCloudExtreCur2->header.stamp, "camera_init", "transformSum"));
}

void laserCloudLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudLast2)
{
	if (laserCloudLast2->header.stamp.toSec() <= timeLaserCloudLast + 0.005)
		return;

	timeLaserCloudLast = laserCloudLast2->header.stamp.toSec();
	startTimeLast = startTimeCur;
	startTimeCur = timeLaserCloudLast - initTime;

	// Swich LLast and Last for corner and surf points
	laserCloudCornerLLast = laserCloudCornerLast;
	laserCloudSurfLLast = laserCloudSurfLast;

	// Reconstruct corner and surf points from incoming laser cloud using magic numbers in 'v'
	pcl::PointCloud<pcl::PointXYZHSV> laserCloudLast;
	pcl::fromROSMsg(*laserCloudLast2, laserCloudLast);
	int laserCloudLastSize = laserCloudLast.points.size();

	laserCloudExtreLast->clear();
	laserCloudCornerLast->clear();
	laserCloudSurfLast->clear();
	for (int i = 0; i < laserCloudLastSize; i++)
	{
		if (fabs(laserCloudLast.points[i].v - 2) < 0.005 || fabs(laserCloudLast.points[i].v + 1) < 0.005)
		{
			laserCloudExtreLast->push_back(laserCloudLast.points[i]);
		}
		if (fabs(laserCloudLast.points[i].v - 2) < 0.005 || fabs(laserCloudLast.points[i].v - 1) < 0.005)
		{
			laserCloudCornerLast->push_back(laserCloudLast.points[i]);
		} 
		if (fabs(laserCloudLast.points[i].v) < 0.005 || fabs(laserCloudLast.points[i].v + 1) < 0.005)
		{
			laserCloudSurfLast->push_back(laserCloudLast.points[i]);
		}
	}

	pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreePointer = kdtreeCornerLLast;
	kdtreeCornerLLast = kdtreeCornerLast;
	kdtreeCornerLast = kdtreePointer;
	kdtreeCornerLast->setInputCloud(laserCloudCornerLast);

	kdtreePointer = kdtreeSurfLLast;
	kdtreeSurfLLast = kdtreeSurfLast;
	kdtreeSurfLast = kdtreePointer;
	kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

	if (timeLasted > 4.0)
	{
		newLaserCloudLast = true;
	}
	
	// From main()
//	float rx, ry, rz, tx, ty, tz;
	AccumulateRotation();

	int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
	for (int i = 0; i < laserCloudCornerLastNum; i++)
	{
		TransformToEnd(&laserCloudCornerLast->points[i], &laserCloudCornerLast->points[i], 
		startTimeLast, startTimeCur);
	}

	int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
	for (int i = 0; i < laserCloudSurfLastNum; i++)
	{
		TransformToEnd(&laserCloudSurfLast->points[i], &laserCloudSurfLast->points[i], 
		startTimeLast, startTimeCur);
	}

	for (int i = 0; i < 6; i++)
	{
		transformRec[i] = transform[i];
		transform[i] = 0;
	}

	sensor_msgs::PointCloud2 laserCloudLastOut;
	pcl::toROSMsg(*laserCloudCornerLast + *laserCloudSurfLast, laserCloudLastOut);
	laserCloudLastOut.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
	laserCloudLastOut.header.frame_id = "/camera";
	pubLaserCloudLast2.publish(laserCloudLastOut);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laserOdometry");
	ros::NodeHandle nh;

	tfBroadcaster = new tf::TransformBroadcaster();

	ros::Subscriber subLaserCloudExtreCur = nh.subscribe<sensor_msgs::PointCloud2> 
	("/laser_cloud_extre_cur", 2, laserCloudExtreCurHandler);

	ros::Subscriber subLaserCloudLast = nh.subscribe<sensor_msgs::PointCloud2> 
	("/laser_cloud_last", 2, laserCloudLastHandler);

	pubLaserCloudLast2 = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 2);
	
	ros::spin();

	return 0;
}
