
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"


extern bool DOWNSIZE_VIS;

bool DOWNSIZE_VIS = true;

using namespace std;

/**
 * @brief: Benchmark - helper function to run a benchmark with a specific set of parameters
 */
int benchmark(	const string &detectorType,
				const string &descriptorType,
				const string &matcherType,
				const string &descriptorTypeM,
				const string &selectorType,
				std::vector< double > &lidarTTCTimes,
				std::vector< double > &camTTCTimes,
				const bool bVis_3DObj = true,
				const bool bVis = true,
                const bool bWait = true
				)
{
    lidarTTCTimes.clear();
    camTTCTimes.clear();

    if ( ( detectorType.compare("AKAZE") != 0 && descriptorType.compare("AKAZE") == 0 ) ||
         ( detectorType.compare("SIFT")  == 0 && descriptorType.compare("ORB")   == 0 ) )
    {
        cerr << __FILE__ << "[" << __LINE__ << "] in function " << __FUNCTION__ << ": detector \"" << detectorType << "\" cannot be combined with descriptor \"" << descriptorType << "\"" << endl;
        return -1;
    }

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        cv::Mat visImg;
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file 
        cv::Mat img = cv::imread(imgFullFilename);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.push_back(frame);

        if (bVis)
        {
            visImg = (dataBuffer.end() - 1)->cameraImg.clone();
        }

        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


        /* DETECT & CLASSIFY OBJECTS */

        float confThreshold = 0.2;
        float nmsThreshold = 0.4;        
        detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                      yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

        cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;


        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file
        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
    
        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

        cout << "#3 : CROP LIDAR POINTS done" << endl;


        /* CLUSTER LIDAR POINT CLOUD */

        // associate Lidar points with camera-based ROI
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

        // Visualize 3D objects
        if(bVis_3DObj)
        {
            show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), false);
        }

        cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;
        
        
        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            detKeypointsModern( keypoints, imgGray, detectorType, false );
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;

        cout << "#5 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#6 : EXTRACT DESCRIPTORS done" << endl;


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorTypeM, matcherType, selectorType);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            
            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
            map<int, int> bbBestMatches;
            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1)); // associate bounding boxes between current and previous frame using keypoint matches
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end()-1)->bbMatches = bbBestMatches;

            cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;


            /* COMPUTE TTC ON OBJECT IN FRONT */

            // loop over all BB match pairs
            for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
            {
                // find bounding boxes associates with current match
                BoundingBox *prevBB, *currBB;
                for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                {
                    if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        currBB = &(*it2);
                    }
                }

                for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                {
                    if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        prevBB = &(*it2);
                    }
                }

                // compute TTC for current match
                if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                {
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar; 
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                    //// EOF STUDENT ASSIGNMENT

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                    //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                    double ttcCamera;
                    clusterKptMatchesWithROI(*prevBB, *currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);                    
                    //// EOF STUDENT ASSIGNMENT

                    computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera, bVis ? &visImg : nullptr);

                    lidarTTCTimes.push_back( ttcLidar );
                    camTTCTimes.push_back( ttcCamera );

                    if (bVis)
                    {
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(prevBB->roi.x, prevBB->roi.y), cv::Point(prevBB->roi.x + prevBB->roi.width, prevBB->roi.y + prevBB->roi.height), cv::Scalar(255, 0, 0), 2);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 0, 255), 2);
                        char str[200];
                        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));
                    }
                } // eof TTC computation
            } // eof loop over all BB matches            
        }
        if (bVis)
        {
            string windowName = "Final Results : TTC";
            cv::namedWindow(windowName, 4);
            //if( DOWNSIZE_VIS ) resize(visImg, visImg, cv::Size(), 0.25, 0.25, cv::INTER_CUBIC);
            cv::imshow(windowName, visImg);
            if( bWait )
            {
                cout << "Press key to continue to next frame" << endl;
                cv::waitKey(0);
            }
        }
    } // eof loop over all images

    return 0;
}


// --------------------------------------------------------------------
// --------------------------------------------------------------------
/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
// 	std::vector<std::string> detectorTypeList = { "SHITOMASI", "HARRIS", "SIFT" };
// 	std::vector<std::string> descriptorTypeList = { "AKAZE", "SIFT" };
	std::vector<std::string> detectorTypeList = { "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT" };
	std::vector<std::string> descriptorTypeList = { "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" };
	std::vector< TTCMethod > ttcMethodList = { TTCMedian, TTCAverage10, TTCAverage10_First10, TTCAverageSmallestError };

	bool bVis_3DObj = !true;        // visualize lidar cloud and ROI box
	bool bVis       = !true;        // visualize camera image
	bool bWait      = bVis;       // wait after each cycle (when bVis is true)
	bool runTTCBenchmark = false;
    bool runDetBenchmark = true;

    const char fprec[] = "%2.2f";  // precision for output numbers

	int cellWidth = 6;
	string tableSeparator = "|";
	// filled: fill the given string with leading spaced to get a resulting string of length "cellWidth"
	// remark: the function is similar to std::string.resize, but it extends the string on the left side instead of the right side.
	std::function< string( string, int ) > filledS = [ ]( string str, int w ) {
		str.insert( str.begin(), std::max( 0, w - static_cast<int>( str.size() ) ), ' ' );
		return str;
	};
	std::function< string( int ) > filledI = [ filledS, cellWidth ]( int i ) { return filledS( (i >= 0) ? to_string( i ) : "-", cellWidth ); };
	std::function< string( double, const char* ) > filledF = [ filledS, cellWidth ]( double f, const char* format = "%f" ) {
        static char ht[64];
        sprintf(ht, format, f);
        return filledS( (f >= 0.0) ? ht : "-", cellWidth );
    };

    if( runTTCBenchmark )
	{
        // remark: use map instead of vector
		std::vector< std::vector< double > > lidarTTCMethodResults;
		std::vector< std::vector< double > > camTTCMethodResults;
		for( TTCMethod ttcMethod : ttcMethodList )
		{
			ttcMethodLidar = ttcMethod;
			ttcMethodCam   = ttcMethod;

			string detectorType = "FAST";
			string descriptorType = "BRIEF";
			string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
			string descriptorTypeM = "DES_BINARY"; // DES_BINARY, DES_HOG
			string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

			std::vector< double > lidarTTCTimes;
			std::vector< double > camTTCTimes;

			int r = benchmark(	detectorType,
								descriptorType,
								matcherType,
								descriptorTypeM,
								selectorType,
								lidarTTCTimes,
								camTTCTimes,
								bVis_3DObj,
								bVis,
								bWait
							);
            // --------------------------------------
            lidarTTCMethodResults.push_back( lidarTTCTimes );
            camTTCMethodResults.push_back( camTTCTimes );
		}
		// --------------------------------------
		int numBenchmarks = lidarTTCMethodResults.front().size();
		for( std::vector< double > &benchResult : lidarTTCMethodResults )
		{
			assert( benchResult.size() == numBenchmarks );
		}
		for( std::vector< double > &benchResult : camTTCMethodResults )
		{
			assert( benchResult.size() == numBenchmarks );
		}
		// --------------------------------------
		for( int b = -2; b < numBenchmarks; b++ )
		{
			// Lidar
			for( int m = 0; m < lidarTTCMethodResults.size(); m++ )
			{
				if( b == -2 )
				{
					if( m > 0 ) cout << " " << tableSeparator;
					cout << filledS( " Lidar:" + ttcMethodStr( ttcMethodList [ m ] ), cellWidth );
// 					cout << filledS( "&nbsp; &nbsp; &nbsp; Lidar:" + ttcMethodStr( ttcMethodList [ m ] ) + " &nbsp; &nbsp; &nbsp; ", cellWidth );
				}
				else if( b == -1 )
				{
					if( m > 0 ) cout << "-" << tableSeparator;
					cout << std::string( cellWidth, '-' );
				}
				else
				{
					if( m > 0 ) cout << " " << tableSeparator ;
					cout << filledF( lidarTTCMethodResults[m][b], fprec );
				}
			}
			// Camera
			for( int m = 0; m < camTTCMethodResults.size(); m++ )
			{
				if( b == -2 )
				{
					cout << " " << tableSeparator;
					cout << filledS( " Camera:" + ttcMethodStr( ttcMethodList [ m ] ), cellWidth );
// 					cout << filledS( "&nbsp; &nbsp; &nbsp; Camera:" + ttcMethodStr( ttcMethodList [ m ] ) + " &nbsp; &nbsp; &nbsp; ", cellWidth );
				}
				else if( b == -1 )
				{
					cout << "-" << tableSeparator;
					cout << std::string( cellWidth, '-' );
				}
				else
				{
					cout << " " << tableSeparator ;
					cout << filledF( camTTCMethodResults[m][b], fprec );
				}
			}
			cout << endl;
		}
	}


	if( runDetBenchmark )
    {
        int numElements = -1;
        std::vector< double > lidarTTCMethodResults;
        std::map< std::pair<string, string>, std::vector< double > > camTTCMethodResults;

        for( string detectorType : detectorTypeList )
        {
            for( string descriptorType : descriptorTypeList )
            {
                //~ TTCMedian, TTCAverage10, TTCAverage10_First10, TTCAverageSmallestError
                ttcMethodLidar = TTCMedian;
                ttcMethodCam   = TTCMedian;
                string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                string descriptorTypeM = "DES_BINARY"; // DES_BINARY, DES_HOG
                string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

                if( descriptorType.compare("SIFT") == 0 )
                {
                    matcherType = "MAT_BF";           // MAT_BF, MAT_FLANN   (BF for benchmark!)
                    descriptorTypeM = "DES_HOG";      // DES_BINARY, DES_HOG
                    selectorType = "SEL_KNN";         // SEL_NN, SEL_KNN
                }

                std::vector< double > lidarTTCTimes;
                std::vector< double > camTTCTimes;
                try {
                    cerr << "Running benchmark with detectorType=\"" << detectorType << "\", descriptorType=\"" << descriptorType << "\"" << endl;
                int r = benchmark(	detectorType,
                                    descriptorType,
                                    matcherType,
                                    descriptorTypeM,
                                    selectorType,
                                    lidarTTCTimes,
                                    camTTCTimes,
                                    bVis_3DObj,
                                    bVis,
                                    bWait
                                );
                }
                catch(const cv::Exception& e)
                {
                    cerr << endl;
                    cerr << "*****************************************************************************" << endl;
                    cerr << "EXCEPTION!" << endl;
                    cerr << "detectorType=\"" << detectorType << "\", descriptorType=\"" << descriptorType << "\"" << endl;
                    cerr << e.msg << endl;
                }
                if( numElements == -1 && lidarTTCTimes.size() > 0 ) // first run
                {
                    numElements = lidarTTCTimes.size();
                    lidarTTCMethodResults = lidarTTCTimes;
                }
                std::cout << __LINE__ << ", detector=" << detectorType << ", descriptor=" << descriptorType <<  ", #el=" << numElements << ", #lidarTTC=" << lidarTTCTimes.size() << ", #camTTC=" << camTTCTimes.size() << std::endl;
                assert( lidarTTCTimes.size() == numElements || lidarTTCTimes.size() == 0 );
                assert( camTTCTimes.size() == numElements || camTTCTimes.size() == 0 );
                camTTCMethodResults[std::pair<string,string>(detectorType, descriptorType)] = camTTCTimes;
            }
        }
        // --------------------------------------
        // setup of table:
        //         Method  | measurement 1-2 | measurement 2-3 | measurement 3-4 | ...
        // ----------------|-----------------|-----------------|-----------------|----
        // Lidar           |    8.2          |       7.6       |      6.5        | ...
        // Cam<SIFT,BRISK> |    8.2          |       7.6       |      6.5        | ...
        // Cam<FAST,FAST>  |    8.2          |       7.6       |      6.5        | ...
        // ...
        int cellWidthT = 16; // width for first column
        cout << filledS( "Method", cellWidthT );
        for( int i = 0; i < numElements; i++ )
        {
            std::stringstream x ( "" );
            x << (i+1) << "-" << (i+2);
            cout << tableSeparator << " " << filledS( x.str(), cellWidth );
        }
        cout << tableSeparator << " " << filledS( "error", cellWidth );
        cout << endl;


        for( int i = 0; i <= numElements + 1; i++ )
        {
            if( i > 0 ) cout << "-" << tableSeparator;
            cout << std::string( ( i > 0 ) ? cellWidth : cellWidthT, '-' );
        }
        cout << endl;


        cout << filledS( "Lidar", cellWidthT );
        for( std::vector< double >::iterator result = lidarTTCMethodResults.begin(); result != lidarTTCMethodResults.end(); result++ )
        {
            cout << " " << tableSeparator << filledF( *result, fprec );
        }
        cout << " " << tableSeparator << filledS( "-", cellWidth );
        cout << endl;


        for( std::map< std::pair<string, string>, std::vector< double > >::iterator results = camTTCMethodResults.begin(); results != camTTCMethodResults.end(); results++ )
        {
            string colName = results->first.first + "/" + results->first.second;
            cout << filledS( colName, cellWidthT );
            if( results->second.size() > 0 )
            {
                std::vector< double >::iterator resultLidar = lidarTTCMethodResults.begin();
                double totalError = 0.0;
                int numMeasurements = 0;
                for( std::vector< double >::iterator result = results->second.begin(); result != results->second.end(); result++ )
                {
                    if( *result < 990 )
                    {
                        cout << " " << tableSeparator << filledF( *result, fprec );
                        assert( resultLidar != lidarTTCMethodResults.end() );
                        double e = (*result - *resultLidar);
                        totalError += e * e;
                        numMeasurements++;
                    }
                    else
                    {
                        cout << " " << tableSeparator << filledS( "-", cellWidth );
                    }
                    resultLidar++;
                }
                totalError = sqrt( totalError / numMeasurements );
                cout << " " << tableSeparator << filledF( totalError, fprec );
            }
            else
            {
                cout << " " << tableSeparator << filledS( "no valid combination of detector / descriptor", cellWidth );
            }
            cout << endl;
        }

        /*
        // print benchmark tables
        if( true )
        {
            ofstream tableFileKeypoints;
            ofstream tableFileMatchedKeypoints;
            ofstream tableFileTime;

            tableFileKeypoints.open("task7_num_keypoints.csv");
            tableFileMatchedKeypoints.open("task8_num_matchedkeypoints.csv");
            tableFileTime.open("task9_time.csv");

            std::stringstream headerSeparator;
            tableFileKeypoints        << filledS("");
            tableFileMatchedKeypoints << filledS("");
            tableFileTime             << filledS("");
            headerSeparator           << ":" << std::string( cellWidth-1, '-' );

            for( string descriptorType : descriptorTypeList )
            {
                tableFileKeypoints        << " " << tableSeparator << filledS( descriptorType );
                tableFileMatchedKeypoints << " " << tableSeparator << filledS( descriptorType );
                tableFileTime             << " " << tableSeparator << filledS( descriptorType );
                headerSeparator           << ":" << tableSeparator << std::string( cellWidth, '-' );
            }
            tableFileKeypoints        << endl;
            tableFileMatchedKeypoints << endl;
            tableFileTime             << endl;

            tableFileKeypoints        << headerSeparator.str() << endl;
            tableFileMatchedKeypoints << headerSeparator.str() << endl;
            tableFileTime             << headerSeparator.str() << endl;
            
            for( string detectorType : detectorTypeList )
            {
                tableFileKeypoints        << filledS( detectorType );
                tableFileMatchedKeypoints << filledS( detectorType );
                tableFileTime             << filledS( detectorType );

                for( string descriptorType : descriptorTypeList )
                {
                    tableFileKeypoints        << " " << tableSeparator << filledI( numKeypointsMap[ detectorType ][ descriptorType ] );
                    tableFileMatchedKeypoints << " " << tableSeparator << filledI( numMatchedKeypointsMap[ detectorType ][ descriptorType ] );
                    tableFileTime             << " " << tableSeparator << filledF( timeMap[ detectorType ][ descriptorType ] );
                }
                tableFileKeypoints        << endl;
                tableFileMatchedKeypoints << endl;
                tableFileTime             << endl;
            }

            tableFileKeypoints.close();
            tableFileMatchedKeypoints.close();
            tableFileTime.close();
        }
        */
    }

    return 0;
}
