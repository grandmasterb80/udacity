/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#ifndef CLUSTER_LIB_H
#define CLUSTER_LIB_H

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

std::vector< std::vector < int > > euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize = 1, int maxSize = INT_MAX);

#endif // CLUSTER_LIB_H
