/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#define USE_CITY_SCENARIO
// #define USE_CITY_ANIMATION

typedef typename pcl::PointCloud< pcl::PointXYZ >::Ptr PointCloudPtr;
typedef typename pcl::PointCloud< pcl::PointXYZI >::Ptr PointICloudPtr;

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
    renderPointCloud( viewer, scanSegments.first, "Plane Segment", cloudColorPlane);
    //renderPointCloud( viewer, scanSegments.second, "Obstacles", cloudColorObstacles[0] );
#ifdef USE_PCL
    static const int maxIterations = 3 + 2 * log( myLidar->PointDensity() );
#else
    static const int maxIterations = 10 + 3 * log( myLidar->PointDensity() );
#endif
    std::pair< PointCloudPtr, PointCloudPtr > scanSegments = myPPC->SegmentPlane(lidarScan, maxIterations, 0.25);

    //renderPointCloud( viewer, scanSegments.first, "Plane Segment", cloudColorPlane);
    renderPointCloud( viewer, scanSegments.second, "Obstacles", cloudColorObstacles[0] );
/*
    std::vector< PointCloudPtr > objectClusters ( myPPC->Clustering( scanSegments.second, 1.5, std::max( 3, (int)( myLidar->PointDensity() / 120 ) ), ( int ) ( myLidar->PointDensity() / 10 ) ) );
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
*/
}


//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* myPPCI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    static const Color cloudColor         ( 0.0, 1.0, 1.0);
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

#ifndef USE_CITY_ANIMATION
    inputCloud = myPPCI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
#endif

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
    
    Eigen::Vector4f minROI( -10.0, -6.0, -3.0, 1 ); // -10, 6
    Eigen::Vector4f maxROI(  30.0,  6.0,  2.0, 1 ); //  30, 6
    pcl::PointCloud< pcl::PointXYZI >::Ptr filterCloud ( myPPCI->FilterCloud( inputCloud, 0.2 , minROI, maxROI ) );
    renderPointCloud( viewer, filterCloud, "filterCloud", cloudColor );

#ifdef USE_PCL
    static const int maxIterations = 10;
#else
    static const int maxIterations = 50;
#endif
    // split data into plane and objects
    std::pair< PointICloudPtr, PointICloudPtr > scanSegments = myPPCI->SegmentPlane( filterCloud, maxIterations, 0.22);

    renderPointCloud( viewer, scanSegments.first, "Plane Segment", cloudColorPlane);
    //renderPointCloud( viewer, scanSegments.second, "Obstacles", cloudColorObstacles[0] );

#ifdef USE_PCL
    static const double clusterTolerance = 0.5;
#else
    static const double clusterTolerance = 5;
#endif
    std::vector< PointICloudPtr > objectClusters ( myPPCI->Clustering( scanSegments.second, clusterTolerance, filterCloud->size() / 400, filterCloud->size() / 10 ) );
    uint32_t j = 0;
    for( PointICloudPtr obj : objectClusters )
    {
        // generate name for the cluster
        std::stringstream objName;
        objName << "Object Cluster " << j;
        j++;

        // render object
        renderPointCloud( viewer, obj, objName.str(), cloudColorObstacles[ j % cloudColorObstacles.size() ] );

        //Box box = myPPCI->BoundingBox( obj );
        BoxQ box = myPPCI->BoundingQBox( obj );
        renderBox( viewer, box, j  );
    }
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
    myPPCI = new ProcessPointClouds< pcl::PointXYZI >();

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // valid values: CameraAngle setAngle = {XY, TopDown, Side, FPS}
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
#ifdef USE_CITY_SCENARIO
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
#ifdef USE_CITY_ANIMATION
    std::vector<boost::filesystem::path> stream = myPPCI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
#else // USE_CITY_ANIMATION
    cityBlock( viewer, myPPCI, inputCloudI );
#endif // USE_CITY_ANIMATION
#else // USE_CITY_SCENARIO
    simpleHighway( viewer);
#endif // USE_CITY_SCENARIO

    while (!viewer->wasStopped ())
    {
#ifdef USE_CITY_SCENARIO
#ifdef USE_CITY_ANIMATION
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = myPPCI->loadPcd( streamIterator->string() );
        cityBlock( viewer, myPPCI, inputCloudI );

        streamIterator++;
        if(streamIterator == stream.end()) // restart when end is reached
        {
            streamIterator = stream.begin();
        }
#endif // USE_CITY_ANIMATION
#endif // USE_CITY_SCENARIO
        viewer->spinOnce ();
    } 

    if( myLidar != nullptr ) { delete myLidar; myLidar = nullptr; }
    if( myPPC != nullptr ) { delete myPPC; myPPC = nullptr; }
    if( myPPCI != nullptr ) { delete myPPCI; myPPCI = nullptr; }
}
