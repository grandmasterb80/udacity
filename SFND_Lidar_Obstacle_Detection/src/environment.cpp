/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

typedef typename pcl::PointCloud< pcl::PointXYZ >::Ptr PointCloudPtr;

Lidar* myLidar;
ProcessPointClouds < pcl::PointXYZ > *myPPC;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    Color cloudColor         ( 0.0, 0.0, 1.0);
    Color cloudColorPlane    ( 0.0, 1.0, 0.0);
    Color cloudColorObstacles( 1.0, 0.0, 0.0);
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    // struct Lidar::Lidar(std::vector<Car> setCars, double setGroundSlope)
    myLidar = new Lidar(cars, 0.0 /*5.0 * pi / 180.0*/);
    PointCloudPtr lidarScan = myLidar->scan();
    //renderRays( viewer, cars[0].position + myLidar->position, lidarScan );
    //void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
    //renderPointCloud( viewer, lidarScan, "Lidar Scan", cloudColor);

    // TODO:: Create point processor
    myPPC = new ProcessPointClouds< pcl::PointXYZ >(); //::ProcessPointClouds() {}
    std::pair< PointCloudPtr, PointCloudPtr > scanSegments = myPPC->SegmentPlane(lidarScan, 3 + log( myLidar->PointDensity() ), 0.22);

    renderPointCloud( viewer, scanSegments.first, "Plane Segment", cloudColorPlane);
    renderPointCloud( viewer, scanSegments.second, "Obstacles", cloudColorObstacles);

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    myLidar = nullptr;
    myPPC = nullptr;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // valid values: CameraAngle setAngle = {XY, TopDown, Side, FPS}
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 

    if( myLidar != nullptr ) { delete myLidar; myLidar = nullptr; }
    if( myPPC != nullptr ) { delete myPPC; myPPC = nullptr; }
}
