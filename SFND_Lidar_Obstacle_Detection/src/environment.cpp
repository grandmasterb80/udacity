/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#define USE_CITY_SCENARIO

typedef typename pcl::PointCloud< pcl::PointXYZ >::Ptr PointCloudPtr;

Lidar* myLidar;
ProcessPointClouds < pcl::PointXYZ > *myPPC;
ProcessPointClouds < pcl::PointXYZI > *myPPCI;

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
    static const Color cloudColor         ( 0.0, 0.0, 1.0);
    static const Color cloudColorPlane    ( 0.0, 1.0, 0.0);
    static const std::vector< Color > cloudColorObstacles (
    {   
        Color( 1.0, 0.0, 0.0),
        Color( 1.0, 1.0, 0.0),
        Color( 1.0, 0.0, 1.0),
        Color( 0.0, 1.0, 1.0),
        Color( 0.6, 0.0, 0.0),
        Color( 0.6, 0.6, 0.0),
        Color( 0.6, 0.0, 0.6),
        Color( 0.0, 0.6, 0.6),
    } );
    
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
    //renderPointCloud( viewer, scanSegments.second, "Obstacles", cloudColorObstacles[0] );

    std::vector< PointCloudPtr > objectClusters ( myPPC->Clustering( scanSegments.second, 1.5, myLidar->PointDensity() / 120, myLidar->PointDensity() / 10 ) );
    uint32_t j = 0;
    for( PointCloudPtr obj : objectClusters )
    {
        // generate name for the cluster
        std::stringstream objName;
        objName << "Object Cluster " << j;
        j++;

        // render object
        renderPointCloud( viewer, obj, objName.str(), cloudColorObstacles[ j % cloudColorObstacles.size() ] );

        //Box box = myPPC->BoundingBox( obj );
        BoxQ box = myPPC->BoundingQBox( obj );
        renderBox( viewer, box, j /*objName.str()*/ );
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    myPPCI = new ProcessPointClouds< pcl::PointXYZI >();
    pcl::PointCloud< pcl::PointXYZI >::Ptr inputCloud ( myPPCI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd") );
    //renderPointCloud( viewer, inputCloud, "inputCloud" );

    // cloud filtering
    // X range for ROI:
    //    - 500m viewing distance (whatever we can get) ==> could be made dynamic on speed / scenario
    //    -  41m backwards (allows to detect vehicles on highway approaching for overtaking 3second at 50kph delta speed to ego)
    // 
    // Y range for ROI:
    //    - lane width for highways: 1.5 * 3.7 ==> +/- 6m
    //    - max curvature: 1/250m (ramp-on / ramp-off):   1 / 250m * (viewing distance)^2 = 10m
    //    ==> Y range = +/- 16m
    //
    // Z range for ROI:
    //    - 2% slope change ==> 10m @ 500m
    //    - truck height ==> 4m
    //    ==> Z range +/-15m
    // resolution
    // ( 100 - (-41) ) * 16 * 2 * 15 * 2 ==> (about) 110000m³ (with 100m actual viewing distance)
    //Eigen::Vector4f minROI( -41.0, -16.0, -15.0, 1 );
    //Eigen::Vector4f maxROI( 500.0,  16.0,  15.0, 1 );
    
    Eigen::Vector4f minROI( -10.0, -8.0, -3.0, 1 );
    Eigen::Vector4f maxROI(  50.0,  8.0,  2.0, 1 );
    pcl::PointCloud< pcl::PointXYZI >::Ptr filterCloud ( myPPCI->FilterCloud( inputCloud, 0.2 , minROI, maxROI ) );
    renderPointCloud( viewer, filterCloud, "filterCloud" );
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
    myPPCI = nullptr;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // valid values: CameraAngle setAngle = {XY, TopDown, Side, FPS}
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
#ifdef USE_CITY_SCENARIO
    cityBlock(viewer);
#else // USE_CITY_SCENARIO
    simpleHighway(viewer);
#endif // USE_CITY_SCENARIO

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 

    if( myLidar != nullptr ) { delete myLidar; myLidar = nullptr; }
    if( myPPC != nullptr ) { delete myPPC; myPPC = nullptr; }
    if( myPPCI != nullptr ) { delete myPPCI; myPPCI = nullptr; }
}
