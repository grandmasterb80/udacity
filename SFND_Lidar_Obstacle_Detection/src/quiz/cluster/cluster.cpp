/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include "cluster_lib.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = ( points[i].size() >= 3 ) ? points[i][2] : 0.0f;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0, uint maxDepth=3)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
        
        float px = node->point[0];
        float py = node->point[1];
        float pz = (maxDepth >= 2) ? node->point[2] : 0.0f;
		// split on x axis
		if( depth % maxDepth == 0 )
		{
            viewer->addLine(pcl::PointXYZ(px, window.y_min, pz),pcl::PointXYZ(px, window.y_max, pz),0,0,1,"line"+std::to_string(iteration*2));
            viewer->addLine(pcl::PointXYZ(px, py, window.z_min),pcl::PointXYZ(px, py, window.z_max),0,0,1,"line"+std::to_string(iteration*2+1));
			lowerWindow.x_max = px;
			upperWindow.x_min = px;
		}
		// split on y axis
		else if( depth % maxDepth == 1 )
		{
            viewer->addLine(pcl::PointXYZ(window.x_min, py, pz),pcl::PointXYZ(window.x_max, py, pz),1,0,0,"line"+std::to_string(iteration*2));
            viewer->addLine(pcl::PointXYZ(px, py, window.z_min),pcl::PointXYZ(px, py, window.z_max),1,0,0,"line"+std::to_string(iteration*2+1));
			lowerWindow.y_max = py;
			upperWindow.y_min = py;
		}
		else if( depth % maxDepth == 2 )
        {
			viewer->addLine(pcl::PointXYZ(window.x_min, py, pz),pcl::PointXYZ(window.x_max, py, pz),1,0,0,"line"+std::to_string(iteration*2));
			viewer->addLine(pcl::PointXYZ(px, window.y_min, pz),pcl::PointXYZ(px, window.z_max, pz),1,0,0,"line"+std::to_string(iteration*2+1));
			lowerWindow.z_max = pz;
			upperWindow.z_min = pz;
        }
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);
    }
}


int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min = -10;
  	window.z_max =  10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search({-6,7},3.0);
  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
