#ifndef RANSAC_LIB_H
#define RANSAC_LIB_H

#include <cstdlib>
#include <unordered_set>

#include <Eigen/Dense>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

//#define PRINT_DEBUG

template<typename PointT>
float LowestZ(typename pcl::PointCloud< PointT >::Ptr &cloud)
{
    float lowestZ = 9999999.0f;
    for( PointT p : *cloud )
    {
        if( p.z < lowestZ ) lowestZ = p.z;
    }
    return lowestZ;
}


template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud< PointT >::Ptr &cloud, int maxIterations, float distanceTol)
{
    // First call: we do static initialization to memorize the last plane that had a good match. We always
    // try this first, since we are slightly moving through the scene.
    // Attention: initialization is application specific: we assume that
    // ground is a X-Y-plane, which includes the lowest points in the cloud
    static Eigen::Vector3f planepoint( 0.0, 0.0, LowestZ<PointT>( cloud ) + distanceTol );
    static Eigen::Vector3f normal( 0.0, 0.0, 1.0 );  // assume that ground is flat
   
	std::unordered_set<int> inliersResult;
    double bestStdDev = 100.0;
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
        int i;
        bool lastIteration = false;
        for( i = 0; i < maxIterations && !lastIteration; i++ )
        {
            std::unordered_set<int> intermediateInliersResult;
            uint32_t p1 = 0;
            uint32_t p2 = 0;
            uint32_t p3 = 0;
            // go to last iteration approach if we already have a specific percentage of the input cloud in the determined plane
            lastIteration = ( i == ( maxIterations - 1 ) ) ||
                            ( inliersResult.size() > cloud->size() / 10 * 5 );
            // bool checkResult = false;
            if( lastIteration && inliersResult.size() > 1000 )
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

                // Eigenvector computation is based on http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
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
                if( i > 0 )
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
            double stdDev = 0.0;
            for( uint32_t j = 0; j < numPoints && numOutliers < numOutliersBest; j++ )
            {
                Eigen::Vector3f p( cloud->at( j ).getArray3fMap() );
                double dist = fabs( p.dot( normal ) - distanceOffset );
                if( dist < distanceTol )
                {
                    stdDev += dist*dist;
                    intermediateInliersResult.insert( j );
                }
                else
                {
                    numOutliers++;
                }
            }
            stdDev /= intermediateInliersResult.size();
            stdDev = sqrt( stdDev );

            if( intermediateInliersResult.size() > inliersResult.size() || 
                ( intermediateInliersResult.size() == inliersResult.size() &&
                stdDev < bestStdDev ) )
            {
#ifdef PRINT_DEBUG
                std::cout << "New inlier data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << (lastIteration ? " in last iteration" : "" ) << std::endl;
#endif
                inliersResult = intermediateInliersResult;
                bestStdDev = stdDev;
            }
        }
#ifdef PRINT_DEBUG
        std::cout << "Plane was computed in " << i << " iterations" << std::endl;
#endif
    }

    //std::cout << "Data set has " << numPoints << "points - found " << inliersResult.size() << " inliers" << std::endl;
	return inliersResult;
}

#endif // RANSAC_LIB_H
