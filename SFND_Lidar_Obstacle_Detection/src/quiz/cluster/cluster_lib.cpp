/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include <unordered_set>
#include "kdtree.h"
#include "cluster_lib.h"

//#define PRINT_DEBUG

std::vector< std::vector < int > > euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	// TODO: Fill out this function to return list of indices for each cluster
    std::vector< std::vector < int > > clusters;
// --------------------- DANIEL SOLUTION START ---------------------
    std::unordered_set< int > visitedPoints;
    for( int currentPoint = 0; currentPoint != points.size(); currentPoint++ )
    {
        std::unordered_set < int > newClusterSet;
        std::vector< int > pointsToCheck;
        if( visitedPoints.count( currentPoint ) > 0 ) continue;
        pointsToCheck.push_back( currentPoint );
        do
        {
            int pIndex = pointsToCheck.back();
            pointsToCheck.pop_back();
            if( visitedPoints.count( pIndex ) > 0 ) continue;
            visitedPoints.insert ( pIndex );
            newClusterSet.insert ( pIndex );
            std::vector< int > pointsToCheckNew ( tree->search( points[ pIndex ], distanceTol ) );
            pointsToCheck.insert( pointsToCheck.end(), pointsToCheckNew.begin(), pointsToCheckNew.end() );
        } while( !pointsToCheck.empty() );

        if( newClusterSet.size() >= minSize && newClusterSet.size() <= maxSize )
        {
#ifdef PRINT_DEBUG
            std::cout << "   * euclideanCluster: found new cluster with size " << newClusterSet.size() << std::endl;
#endif // PRINT_DEBUG
            std::vector < int > newCluster( newClusterSet.begin(), newClusterSet.end() );
            clusters.push_back( newCluster );
        }
#ifdef PRINT_DEBUG
        else
        {
            std::cout << "   * euclideanCluster: neglecting new cluster with size " << newClusterSet.size() << std::endl;
        }
#endif // PRINT_DEBUG
    }
#ifdef PRINT_DEBUG
    std::cout << "euclideanCluster: found " << clusters.size() << " clusters with size {";
    for( std::vector<int> c : clusters )
    {
        std::cout << c.size() << ", ";
    }
    std::cout << "}" << std::endl;
#endif // PRINT_DEBUG
// --------------------- DANIEL SOLUTION END ---------------------

	return clusters;
}
