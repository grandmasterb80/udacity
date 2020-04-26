#ifndef RANSAC_LIB_H
#define RANSAC_LIB_H

#include <cstdlib>
#include <unordered_set>

#include <Eigen/Dense>
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

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud< PointT >::Ptr &cloud, int maxIterations, float distanceTol)
{
    static Eigen::Vector3f planepoint;
    static Eigen::Vector3f normal;
    static bool firstRun = true; // used to reuse planepoint and normal from last execution
   
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
            // go to last iteration approach if we already have 60% of all points
            if( inliersResult.size() > cloud->size() / 10 * 6 )
            {
                i = maxIterations - 1;
            }
            // bool checkResult = false;
            if( i == ( maxIterations - 1 ) && inliersResult.size() > 1000 )
            {
                // create point cloud with detected set
                typename pcl::PointCloud< PointT >::Ptr subset ( new pcl::PointCloud< PointT > );
                Eigen::Vector3f center( 0.0, 0.0, 0.0 );
                int numPoints = 0;
                int ii = 0;
                for( int p_index : inliersResult )
                {
                    ii++;
                    if( ( ii & 15 ) != 0 ) continue;
                    numPoints++;
                    Eigen::Vector3f p = (*cloud)[ p_index ].getArray3fMap();
                    center += p;
                    subset->push_back( (*cloud)[ p_index ] );
                }
                center *= 1.0 / numPoints;

                // check number of points in percent: if > 30%, let's try to guess a new plane based on eigenvectors
                typename pcl::PointCloud< PointT > cloudPCAprojection;
                typename pcl::PCA< PointT > pca;
                pca.setInputCloud( subset );
                pca.project( *subset, cloudPCAprojection );

                // Compute principal directions
                Eigen::Vector4f pcaCentroid;
                pcl::compute3DCentroid( *subset, pcaCentroid );
                Eigen::Matrix3f covariance;
                computeCovarianceMatrixNormalized( *subset, pcaCentroid, covariance );
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver( covariance, Eigen::ComputeEigenvectors );
                Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
                // ensure to have perpendicular axes in right-handed system ==> we can assume that Z is already on the z-axis but the sign might change
                //eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross( eigenVectorsPCA.col(1) );
                normal = eigenVectorsPCA.col(2).normalized(); //eigenVectorsPCA.col(0).cross( eigenVectorsPCA.col(1) ).normalized();
                planepoint = center;

                //checkResult = true;
            }
            else
            {
                if( firstRun || i > 0 )
                {
                    do {
                        p1 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
                        p2 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
                        p3 = std::rand() / ( ( RAND_MAX + 1u ) / numPoints );
                    } while( p1 == p2 || p1 == p3 || p2 == p3 );

                    Eigen::Vector3f point2( cloud->at( p2 ).getArray3fMap() );
                    Eigen::Vector3f point3( cloud->at( p3 ).getArray3fMap() );

                    planepoint = cloud->at( p1 ).getArray3fMap();
                    normal = ( planepoint - point2 ).cross( planepoint - point3 ).normalized();
                }
            }

            float distanceOffset = planepoint.dot( normal );

            uint32_t numOutliersBest = numPoints - inliersResult.size();
            uint32_t numOutliers = 0;
            for( uint32_t j = 0; j < numPoints && numOutliers < numOutliersBest; j++ )
            {
                Eigen::Vector3f p( cloud->at( j ).getArray3fMap() );
                if( fabs( p.dot( normal ) - distanceOffset ) < distanceTol )
                {
                    intermediateInliersResult.insert( j );
                }
                else
                {
                    numOutliers++;
                }
            }
/*
            if( checkResult )
            {
                if( intermediateInliersResult.size() > inliersResult.size() )
                {
                    std::cout << "***************** GOOD ************************" << std::endl;
                }
                else
                {
                    std::cout << "------------------ BAD -------------------------" << std::endl;
                }
            }
*/

            if( intermediateInliersResult.size() > inliersResult.size() )
            {
                //std::cout << "New inlier data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << std::endl;
                inliersResult = intermediateInliersResult;
            }
        }
    }
    firstRun = false;

    //std::cout << "Data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << std::endl;
	return inliersResult;
}

#endif // RANSAC_LIB_H
