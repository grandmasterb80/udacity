// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "render/box.h"
#include <Eigen/Geometry> 
#include <pcl/common/pca.h>
#include <pcl/filters/conditional_removal.h>
#include <unordered_set>
#include "quiz/ransac/ransac_lib.h"
#include "quiz/cluster/kdtree.h"
#include "quiz/cluster/cluster_lib.h"

//#define PRINT_DEBUG

/*
#include <pcl/impl/point_types.hpp>

// should be actually defined in pcl/impl/point_types.hpp according to http://docs.pointclouds.org/trunk/structpcl_1_1_point_x_y_z_i.html
inline pcl::PointXYZI PointXYZI_(float _x, float _y, float _z, float i = 0.0) {
    pcl::PointXYZI p;
    p.x = _x;
    p.y = _y;
    p.z = _z;
    p.intensity = i;
    return p;
}
*/

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
// --------------------- DANIEL SOLUTION START ---------------------
    // Step 1: downsample the data
    // Step 2: remove non-ROI data
    // This order is faster than the other way around (5millisec vs. 8millisec)

    typename pcl::PointCloud< PointT >::Ptr filteredCloud ( new pcl::PointCloud< PointT > );
    // Step 1: downsample the data
    // code based on example taken from: http://pointclouds.org/documentation/tutorials/voxel_grid.php
    // Create the filtering object
    //std::cout << "PointCloud before filtering: " << filteredCloud->width * filteredCloud->height << " data points (" << pcl::getFieldsList ( *filteredCloud ) << ")." << std::endl;
    pcl::VoxelGrid < PointT > sor;
    sor.setInputCloud ( cloud );
    sor.setLeafSize ( filterRes, filterRes, filterRes );
    sor.filter ( *filteredCloud );
    //std::cout << "PointCloud after filtering: " << filteredCloud->width * filteredCloud->height << " data points (" << pcl::getFieldsList ( *filteredCloud ) << ")." << std::endl;

    // Step 2: remove non-ROI data
    // setup filter for ego roof
    Eigen::Vector3f minEgo( -3.0f, -1.5f, -1.5f );
    Eigen::Vector3f maxEgo(  3.0f,  1.5f, 0.0f );
    typename pcl::ConditionOr< PointT >::Ptr  EGO_remove_roof  ( new pcl::ConditionOr< PointT > () ); 
    EGO_remove_roof->addComparison (typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "x", pcl::ComparisonOps::LT, minEgo[ 0 ] ) ) );
    EGO_remove_roof->addComparison (typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "x", pcl::ComparisonOps::GT, maxEgo[ 0 ] ) ) );
    EGO_remove_roof->addComparison (typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "y", pcl::ComparisonOps::LT, minEgo[ 1 ] ) ) );
    EGO_remove_roof->addComparison (typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "y", pcl::ComparisonOps::GT, maxEgo[ 1 ] ) ) );
    EGO_remove_roof->addComparison (typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "z", pcl::ComparisonOps::LT, minEgo[ 2 ] ) ) );
    EGO_remove_roof->addComparison (typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "z", pcl::ComparisonOps::GT, maxEgo[ 2 ] ) ) );

    // setup filter for ROI (includes also the "outside of ego box filter defined above)
    typename pcl::ConditionAnd< PointT >::Ptr ROI_cond_and_EGO ( new pcl::ConditionAnd< PointT > () );
    ROI_cond_and_EGO->addComparison ( typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "x", pcl::ComparisonOps::GT, minPoint[ 0 ] ) ) );
    ROI_cond_and_EGO->addComparison ( typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "x", pcl::ComparisonOps::LT, maxPoint[ 0 ] ) ) );
    ROI_cond_and_EGO->addComparison ( typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "y", pcl::ComparisonOps::GT, minPoint[ 1 ] ) ) );
    ROI_cond_and_EGO->addComparison ( typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "y", pcl::ComparisonOps::LT, maxPoint[ 1 ] ) ) );
    ROI_cond_and_EGO->addComparison ( typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "z", pcl::ComparisonOps::GT, minPoint[ 2 ] ) ) );
    ROI_cond_and_EGO->addComparison ( typename pcl::FieldComparison<PointT>::Ptr ( new pcl::FieldComparison< PointT > ( "z", pcl::ComparisonOps::LT, maxPoint[ 2 ] ) ) );
    ROI_cond_and_EGO->addCondition ( EGO_remove_roof );

    typename pcl::ConditionalRemoval< PointT > range_filt;
    range_filt.setInputCloud( filteredCloud );
    //range_filt.setInputCloud( cloud );
    range_filt.setCondition ( ROI_cond_and_EGO );
    range_filt.filter( *filteredCloud );

/*
    // variant: remove ego roof points
    CropBox cropBox( false );
    cropBox.setMin( minEgo );
    cropBox.setMax( maxEgo );
    cropBox.filter( cloud );
*/
/*
    // variant: filter out points outside of ROI
    pcl::PassThrough< pcl::PointT > filter;
    filter.setInputCloud( cloud );
    filter.setFilterFieldName ( "x" );
    filter.setFilterLimits ( minX, maxX);
    filter.setFilterFieldName ( "y" );
    filter.setFilterLimits ( minY, maxY);
    filter.setFilterFieldName ( "z" );
    filter.setFilterLimits ( minZ, maxZ );
    filter.filter ( *filteredCloud );
*/
// --------------------- DANIEL SOLUTION END ---------------------

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
#ifdef PRINT_DEBUG
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
#endif

    return filteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(const InliersDataType &inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
// --------------------- DANIEL SOLUTION START ---------------------
    typename pcl::PointCloud< PointT >::Ptr plane ( new pcl::PointCloud < PointT > );
    typename pcl::PointCloud< PointT >::Ptr obstacles ( new pcl::PointCloud < PointT > );

#ifdef USE_PCL   // PCL ------------------------------------
    // PCL solution for reference
    // Create the filtering object
    pcl::ExtractIndices< PointT > extract;
    // Extract the inliers
    extract.setInputCloud ( cloud );
    extract.setIndices ( inliers );
    extract.setNegative ( false );
    extract.filter ( *plane );
    std::cerr << "PointCloud representing the planar component: " << plane->width * plane->height << " data points." << std::endl;
// Create the filtering object
    extract.setNegative ( true );
    extract.filter ( *obstacles );
#else // PCL ------------------------------------
	for(int index = 0; index < cloud->points.size(); index++)
	{
		if(inliers.count(index))
        {
			plane->points.push_back( cloud->points[index] );
        }
		else
        {
			obstacles->points.push_back( cloud->points[index] );
        }
	}
#endif // PCL ------------------------------------
// --------------------- DANIEL SOLUTION END ---------------------

    std::pair< typename pcl::PointCloud < PointT >::Ptr, typename pcl::PointCloud< PointT >::Ptr > segResult ( plane, obstacles );
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
#ifdef PRINT_DEBUG
    std::cout << "Clustering pointcloud to plane with " << cloud->size() << " points using max " << maxIterations << " iterations and a threshold of " << distanceThreshold << std::endl;
#endif

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
// --------------------- DANIEL SOLUTION START ---------------------
#ifdef USE_PCL
    // PCL solution for reference
    InliersDataType inliers ( new pcl::PointIndices () );
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // Create the segmentation object
    typename pcl::SACSegmentation< PointT > seg;
    // Optional
    seg.setOptimizeCoefficients ( true );
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations ( maxIterations );
    seg.setDistanceThreshold ( distanceThreshold );

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud ( cloud );
    seg.segment ( *inliers, *coefficients );
    if ( inliers->indices.size () == 0 )
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
#else // USE_PCL
    InliersDataType inliers = RansacPlane<PointT>( cloud, maxIterations, distanceThreshold );
    if ( inliers.size () == 0 )
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
#endif // USE_PCL
// --------------------- DANIEL SOLUTION END ---------------------

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
#ifdef PRINT_DEBUG
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
#endif

    std::pair<typename pcl::PointCloud< PointT >::Ptr, typename pcl::PointCloud< PointT >::Ptr> segResult = SeparateClouds( inliers, cloud );
    // segResult.first contains the plane
    // segResult.second contains the outliers, which are expected to be the (yet unclustered) objects

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud< PointT >::Ptr> ProcessPointClouds< PointT >::Clustering(typename pcl::PointCloud< PointT >::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
#ifdef PRINT_DEBUG
    std::cout << "Clustering pointcloud to objects with " << cloud->size() << " points into clusters of size " << clusterTolerance << ", min #points " << minSize <<  " max #points " << maxSize << std::endl;
#endif

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud< PointT >::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
// --------------------- DANIEL SOLUTION START ---------------------
    // PCL solution for reference
#ifdef USE_PCL // PCL ----------------------------------------------------------------------------------------
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree< PointT >::Ptr tree ( new pcl::search::KdTree< PointT > );
    tree->setInputCloud ( cloud );

    typename pcl::EuclideanClusterExtraction< PointT > ec;
    ec.setClusterTolerance ( clusterTolerance ); // kind of the size
    ec.setMinClusterSize ( minSize ); // minimal number of points for considering any cluster an object cluster
    ec.setMaxClusterSize ( maxSize ); // maximal number of points for considering any cluster an object cluster (to consider to separate big clusters into multiple objects)
    ec.setSearchMethod ( tree );      // KD tree created above
    ec.setInputCloud ( cloud );

    // now the actual clustering
    std::vector< pcl::PointIndices > cluster_indices;
    ec.extract ( cluster_indices );
    
    // extract the points into dedicated sets of points (instead of using the set of point indicies)
    int j = 0;
    for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it )
    {
        typename pcl::PointCloud< PointT >::Ptr cloud_cluster ( new pcl::PointCloud< PointT > );
        for ( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit )
        {
            cloud_cluster->points.push_back ( cloud->points[ *pit ] ); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster " << j << ": " << cloud_cluster->points.size () << " data points." << std::endl;
        clusters.push_back( cloud_cluster );
        j++;
    }
#else // PCL ----------------------------------------------------------------------------------------
    // convert points in cloud to vectors & add them to the tree
	std::vector< std::vector < float > > cloudP;
	KdTree tree;
    for (int i=0; i < cloud->size(); i++) 
    {
        std::vector < float > p( { (*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z } );
        cloudP.push_back( p );
    	tree.insert( p, i );
    }

    std::vector< std::vector < int > > cluster_indices ( euclideanCluster( cloudP, &tree, clusterTolerance, minSize, maxSize ) );

    // extract the points into dedicated sets of points (instead of using the set of point indicies)
    int j = 0;
    for ( std::vector< std::vector < int > >::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it )
    {
        typename pcl::PointCloud< PointT >::Ptr cloud_cluster ( new pcl::PointCloud< PointT > );
        for ( std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit )
        {
            cloud_cluster->points.push_back ( cloud->points[ *pit ] ); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

#ifdef PRINT_DEBUG
        std::cout << "PointCloud representing the Cluster " << j << ": " << cloud_cluster->points.size () << " data points." << std::endl;
#endif
        clusters.push_back( cloud_cluster );
        j++;
    }
#endif // PCL ----------------------------------------------------------------------------------------
// --------------------- DANIEL SOLUTION END ---------------------

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
#ifdef PRINT_DEBUG
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
#endif

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingQBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    assert( cluster->size() >= 3 );

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    typename pcl::PointCloud< PointT >::Ptr cloudPCAprojection ( new pcl::PointCloud< PointT > );
    typename pcl::PCA< PointT > pca;
    typename pcl::PointCloud< PointT >::Ptr cluster2D ( new pcl::PointCloud< PointT > ); // clone the cluster and make Z=0 for all points
    {
        // copy all elements into the cluster2D array and set Z=0 to move all points into the X-Y-Plane
        for( PointT &p : *cluster )
        {
            PointT p2d ( p );
            p2d.z = 0.0;
            cluster2D->push_back( p2d );
        }
    }
    pca.setInputCloud( cluster2D );
    pca.project( *cluster2D, *cloudPCAprojection );

    // Eigenvector computation is based on http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid( *cluster2D, pcaCentroid );
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized( *cluster2D, pcaCentroid, covariance );
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver( covariance, Eigen::ComputeEigenvectors );
    //Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
    // ensure to have perpendicular axes in right-handed system ==> we can assume that Z is already on the z-axis but the sign might change
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross( eigenVectorsPCA.col(1) );
    //std::cerr << std::endl << "EigenVectors: " << eigenVectorsPCA << std::endl;
    //std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform( Eigen::Matrix4f::Identity() );
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.0f * ( projectionTransform.block<3,3> ( 0, 0 ) * pcaCentroid.head<3> () );
    typename pcl::PointCloud< PointT >::Ptr cloudPointsProjected ( new pcl::PointCloud< PointT > );
    pcl::transformPointCloud( *cluster, *cloudPointsProjected, projectionTransform );
    // Get the minimum and maximum points of the transformed cloud.
    pcl::getMinMax3D( *cloudPointsProjected, minPoint, maxPoint );
    const Eigen::Vector3f meanDiagonal = 0.5f * ( maxPoint.getVector3fMap() + minPoint.getVector3fMap() );


    BoxQ box;
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    box.bboxQuaternion = eigenVectorsPCA;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
#ifdef PRINT_DEBUG
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;
#endif

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
