#ifndef RANSAC_LIB_H
#define RANSAC_LIB_H

#include <cstdlib>
#include <unordered_set>

// #include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/common/transforms.h>
// #include <iostream> 
// #include <vector>
#include <pcl/common/common.h>

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud< PointT >::Ptr &cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
    uint32_t numPoints = cloud->size();
    if( numPoints <= 3 )
    {
        std::cerr << "Data set has only " << numPoints << " points ==> All points are considered as inliers" << std::endl;
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
            uint32_t p3 = 0;
            do {
                p1 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
                p2 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
                p3 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
            } while( p1 == p2 || p1 == p3 || p2 == p3 );
            PointT delta1;
            PointT delta2;
            PointT normal;

            delta1.getArray3fMap() = cloud->at( p1 ).getArray3fMap() - cloud->at( p2 ).getArray3fMap();
            delta2.getArray3fMap() = cloud->at( p1 ).getArray3fMap() - cloud->at( p3 ).getArray3fMap();
            normal.x = delta1.y * delta2.z - delta1.z * delta2.y;
            normal.y = delta1.z * delta2.x - delta1.x * delta2.z;
            normal.z = delta1.x * delta2.y - delta1.y * delta2.x;
            float nlength = sqrt( normal.x * normal.x + normal.y * normal.y + normal.z * normal.z );
            normal.getArray3fMap() /= nlength;
            
            uint32_t numOutliersBest = numPoints - inliersResult.size();
            uint32_t numOutliers = 0;
            for( uint32_t j = 0; j < numPoints && numOutliers < numOutliersBest; j++ )
            {
                if( j == p1 || j == p2  || j == p3)
                {
                    intermediateInliersResult.insert( j );
                }
                else
                {
                    PointT p;
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
                //std::cout << "New inlier data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << std::endl;
                inliersResult = intermediateInliersResult;
            }
        }
    }
    //std::cout << "Data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << std::endl;
	return inliersResult;
}

#endif // RANSAC_LIB_H
