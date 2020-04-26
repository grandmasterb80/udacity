/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#ifndef CLUSTER_LIB_H
#define CLUSTER_LIB_H

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

std::vector< std::vector < int > > euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
    std::vector< std::vector < int > > clusters;
// --------------------- DANIEL SOLUTION START ---------------------
    std::set< int > visitedPoints;
    for( int currentPoint = 0; currentPoint != points.size(); currentPoint++ )
    {
        std::set < int > newClusterSet;
        std::vector< int > pointsToCheck;
        if( visitedPoints.count( currentPoint ) > 0 ) continue;
        pointsToCheck.push_back( currentPoint );
        do
        {
            int pIndex = pointsToCheck.back();
            pointsToCheck.pop_back();
            if( visitedPoints.count( pIndex ) > 0 ) continue;
            visitedPoints.insert ( pIndex );
            newClusterSet.insert( pIndex );
            std::vector< int > pointsToCheckNew = tree->search( points[ pIndex ], distanceTol );
            pointsToCheck.insert( pointsToCheck.end(), pointsToCheckNew.begin(), pointsToCheckNew.end() );
        } while( !pointsToCheck.empty() );

        std::vector < int > newCluster( newClusterSet.begin(), newClusterSet.end() );
        clusters.push_back( newCluster );
    }
    std::cout << "euclideanCluster: found " << clusters.size() << " clusters with size {";
    for( std::vector<int> c : clusters )
    {
        std::cout << c.size() << ", ";
    }
    std::cout << "}" << std::endl;
// --------------------- DANIEL SOLUTION END ---------------------

	return clusters;
}

#endif // CLUSTER_LIB_H
