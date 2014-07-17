#include "LaserOdometry.h"

#include <math.h>

using namespace loam;

void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                        float &ox, float &oy, float &oz)
{
	float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
	ox = -asin(srx);

	float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
		+ sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
	float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
		- cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
	oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

	float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
		+ sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
	float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
		- cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
	oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

LaserOdometry::LaserOdometry()
{
	initTime = 0;
	
	// This seems to be the actual odometry transformation
	transform[0] = 0;
	transform[1] = 0;
	transform[2] = 0;
	transform[3] = 0;
	transform[4] = 0;
	transform[5] = 0;
	
	// What are these?
	transformRec[0] = 0;
	transformRec[1] = 0;
	transformRec[2] = 0;
	transformRec[3] = 0;
	transformRec[4] = 0;
	transformRec[5] = 0;
	
	transformSum[0] = 0;
	transformSum[1] = 0;
	transformSum[2] = 0;
	transformSum[3] = 0;
	transformSum[4] = 0;
	transformSum[5] = 0;
}

LaserOdometry::~LaserOdometry()
{
	
}

void LaserOdometry::addCurrentSweep(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudIn, double timestamp)
{	
	if (initTime == 0)
	{
		initTime = timestamp;
	}

	timeLasted = timestamp - initTime;

	float s = (timeLasted - timeLastedRec) / (startTimeCur - startTimeLast);
	for (int i = 0; i < 6; i++)
	{
		transform[i] += s * transformRec[i];        
	}
}

void LaserOdometry::addLastSweep(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudIn, double timestamp)
{
	float rx, ry, rz, tx, ty, tz;
	AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);
/*
	float x1 = cos(rz) * (transform[3] - imuShiftFromStartXLast) - sin(rz) * (transform[4] - imuShiftFromStartYLast);
	float y1 = sin(rz) * (transform[3] - imuShiftFromStartXLast) + cos(rz) * (transform[4] - imuShiftFromStartYLast);
	float z1 = transform[5] * 1.05 - imuShiftFromStartZLast;
	
	float x2 = x1;
	float y2 = cos(rx) * y1 - sin(rx) * z1;
	float z2 = sin(rx) * y1 + cos(rx) * z1;

	tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
	ty = transformSum[4] - y2;
	tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);
*/

	// =====================================================================================
	// Ok, this is getting freaking crazy now...
	// We restore the old corner and surface features extracted within ScanRegistration
	// from the values stored in the HSV-channels of the received Pointcloud
	laserCloudExtreLast.clear();
	laserCloudCornerLast.clear();
	laserCloudSurfLast.clear();
	
	int laserCloudLastSize = laserCloudIn->points.size();
	for (int i = 0; i < laserCloudLastSize; i++)
	{
		if (fabs(laserCloudIn->points[i].v - 2) < 0.005 || fabs(laserCloudIn->points[i].v + 1) < 0.005)
		{
			laserCloudExtreLast.push_back(laserCloudIn->points[i]);
		}
		
		if (fabs(laserCloudIn->points[i].v - 2) < 0.005 || fabs(laserCloudIn->points[i].v - 1) < 0.005)
		{
			laserCloudCornerLast.push_back(laserCloudIn->points[i]);
		} 
		
		if (fabs(laserCloudIn->points[i].v) < 0.005 || fabs(laserCloudIn->points[i].v + 1) < 0.005)
		{
			laserCloudSurfLast.push_back(laserCloudIn->points[i]);
		}
/*		
		if (fabs(laserCloudIn->points[i].v - 10) < 0.005)
		{
			imuPitchStartLast = laserCloudIn->points[i].x;
			imuYawStartLast = laserCloudIn->points[i].y;
			imuRollStartLast = laserCloudIn->points[i].z;
		}
		
		if (fabs(laserCloudIn->points[i].v - 11) < 0.005)
		{
			imuPitchLast = laserCloudIn->points[i].x;
			imuYawLast = laserCloudIn->points[i].y;
			imuRollLast = laserCloudIn->points[i].z;
		}
		
		if (fabs(laserCloudIn->points[i].v - 12) < 0.005)
		{
			imuShiftFromStartXLast = laserCloudIn->points[i].x;
			imuShiftFromStartYLast = laserCloudIn->points[i].y;
			imuShiftFromStartZLast = laserCloudIn->points[i].z;
		}
		
		if (fabs(laserCloudIn->points[i].v - 13) < 0.005)
		{
			imuVeloFromStartXLast = laserCloudIn->points[i].x;
			imuVeloFromStartYLast = laserCloudIn->points[i].y;
			imuVeloFromStartZLast = laserCloudIn->points[i].z;
		}
*/
	}
	
	// =====================================================================================

	int laserCloudCornerLastNum = laserCloudCornerLast.points.size();
	for (int i = 0; i < laserCloudCornerLastNum; i++)
	{
		transformToEnd(&laserCloudCornerLast.points[i], &laserCloudCornerLast.points[i], 
		startTimeLast, startTimeCur);
	}
}

void LaserOdometry::transformReset()
{
	for (int i = 0; i < 6; i++)
	{
		transformRec[i] = transform[i];
		transform[i] = 0;
	}
}

void LaserOdometry::transformToStart(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
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

void LaserOdometry::transformToEnd(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
{
/*
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

  float x7 = cos(imuRollStartLast) * (x6 - imuShiftFromStartXLast) 
           - sin(imuRollStartLast) * (y6 - imuShiftFromStartYLast);
  float y7 = sin(imuRollStartLast) * (x6 - imuShiftFromStartXLast) 
           + cos(imuRollStartLast) * (y6 - imuShiftFromStartYLast);
  float z7 = z6 - imuShiftFromStartZLast;

  float x8 = x7;
  float y8 = cos(imuPitchStartLast) * y7 - sin(imuPitchStartLast) * z7;
  float z8 = sin(imuPitchStartLast) * y7 + cos(imuPitchStartLast) * z7;

  float x9 = cos(imuYawStartLast) * x8 + sin(imuYawStartLast) * z8;
  float y9 = y8;
  float z9 = -sin(imuYawStartLast) * x8 + cos(imuYawStartLast) * z8;

  float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
  float y10 = y9;
  float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

  float x11 = x10;
  float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
  float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

  po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
  po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
  po->z = z11;
  po->h = pi->h;
  po->s = pi->s;
  po->v = pi->v;
   */
}
