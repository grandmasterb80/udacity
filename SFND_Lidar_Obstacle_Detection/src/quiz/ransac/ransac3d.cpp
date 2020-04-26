/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <cstdlib>

#include "../../render/render.h"
#include <unordered_set>
#include "ransac_lib.h"

#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
 	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
    uint32_t numPoints = cloud->size();
    if( numPoints <= 2 )
    {
        cout << "Data set has only " << numPoints << " points ==> All points are considered as inliers" << endl;
        for( int i = 0; i < numPoints; i++ )
        {
            inliersResult.insert( i );
        }
    }
    else
    {
        // TODO: Fill in this function
        for( int i = 0; i < maxIterations; i++ )
        {
            std::unordered_set<int> intermediateInliersResult;
            uint32_t p1 = 0;
            uint32_t p2 = 0;
            do {
                p1 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
                p2 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
            } while( p1 == p2 );
            pcl::PointXYZ delta;
            pcl::PointXYZ normal;

            delta.getArray3fMap() = cloud->at( p1 ).getArray3fMap() - cloud->at( p2 ).getArray3fMap();
            // cross product delta x (0,0,1)
            normal.x = delta.y;
            normal.y = -delta.x;
            normal.z = delta.z;
            float nlength = sqrt( normal.x * normal.x + normal.y * normal.y + normal.z * normal.z );
            normal.getArray3fMap() /= nlength;
            
            uint32_t numOutliersBest = numPoints - inliersResult.size();
            uint32_t numOutliers = 0;
            for( uint32_t j = 0; j < numPoints && numOutliers < numOutliersBest; j++ )
            {
                if( j == p1 || j == p2 )
                {
                    intermediateInliersResult.insert( j );
                }
                else
                {
                    pcl::PointXYZ p;
                    p.getArray3fMap() = cloud->at( j ).getArray3fMap() - cloud->at( p1 ).getArray3fMap();
                    p.getArray3fMap() *= normal.getArray3fMap();
                    if( fabs( p.x + p.y + p.z ) < distanceTol )
                    {
                        intermediateInliersResult.insert( j );
                    }
                    else
                    {
                        numOutliers++;
                    }
                }
            }
            if( intermediateInliersResult.size() > inliersResult.size() )
            {
                cout << "New inlier data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << endl;
                inliersResult = intermediateInliersResult;
            }
        }
    }
    cout << "Data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << endl;
	return inliersResult;
}

// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
// 	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.8);

    // Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers ( RansacPlane<pcl::PointXYZ>(cloud, 5, 0.4f) );

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
