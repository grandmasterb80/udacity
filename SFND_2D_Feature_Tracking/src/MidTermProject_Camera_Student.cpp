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

using namespace std;

void runBenchmark(string &detectorType, string &descriptorType, int &numKeypoints, int &numMatchedKeypoints, double &time);

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    std::vector<std::string> detectorTypeList = { "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT" };
    std::vector<std::string> descriptorTypeList = { "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" };
    std::map< std::string, std::map < std::string, int    > > numKeypointsMap;            // task #7
    std::map< std::string, std::map < std::string, int    > > numMatchedKeypointsMap;     // task #8
    std::map< std::string, std::map < std::string, double > > timeMap;                    // task #9
    
    for( string detectorType : detectorTypeList )
    {
        for( string descriptorType : descriptorTypeList )
        {
            int     numKeypoints = 0;
            int     numMatchedKeypoints = 0;
            double  time = 0.0;

            cout << "--------------------------------------------------------------------------------" << endl;
            cout << "Running " << __FUNCTION__ << " with detectorType=\"" << detectorType << "\", descriptorType=\"" << descriptorType << "\"" << endl;

            try {
            runBenchmark( detectorType, descriptorType, numKeypoints, numMatchedKeypoints, time );
            } catch(const cv::Exception& e)
            {
                cout << endl << "***************************************************************************** detectorType=\"" << detectorType << "\", descriptorType=\"" << descriptorType << "\"" << endl;
                cout << e.msg << endl;
            }

            numKeypointsMap[ detectorType ][ descriptorType ]        = numKeypoints;
            numMatchedKeypointsMap[ detectorType ][ descriptorType ] = numMatchedKeypoints;
            timeMap[ detectorType ][ descriptorType ]                = time;
        }
        /*
        numKeypointsMap[ detectorType ] /= descriptorTypeList.size();
        timeMap[ detectorType ] /= descriptorTypeList.size();
        */
    }

    // print benchmark tables
    if( true )
    {
        ofstream tableFileKeypoints;
        ofstream tableFileMatchedKeypoints;
        ofstream tableFileTime;

        tableFileKeypoints.open("task7_num_keypoints.csv");
        tableFileMatchedKeypoints.open("task8_num_matchedkeypoints.csv");
        tableFileTime.open("task9_time.csv");

        tableFileKeypoints        << "   ";
        tableFileMatchedKeypoints << "   ";
        tableFileTime             << "   ";

        for( string descriptorType : descriptorTypeList )
        {
            tableFileKeypoints        << "; " << descriptorType << " ";
            tableFileMatchedKeypoints << "; " << descriptorType << " ";
            tableFileTime             << "; " << descriptorType << " ";
        }
        tableFileKeypoints        << endl;
        tableFileMatchedKeypoints << endl;
        tableFileTime             << endl;

        for( string detectorType : detectorTypeList )
        {
            tableFileKeypoints        << detectorType << " ";
            tableFileMatchedKeypoints << detectorType << " ";
            tableFileTime             << detectorType << " ";

            for( string descriptorType : descriptorTypeList )
            {
                tableFileKeypoints        << "; " << numKeypointsMap[ detectorType ][ descriptorType ] << " ";
                tableFileMatchedKeypoints << "; " << numMatchedKeypointsMap[ detectorType ][ descriptorType ] << " ";
                tableFileTime             << "; " << timeMap[ detectorType ][ descriptorType ] << " ";
            }
            tableFileKeypoints        << endl;
            tableFileMatchedKeypoints << endl;
            tableFileTime             << endl;
        }

        tableFileKeypoints.close();
        tableFileMatchedKeypoints.close();
        tableFileTime.close();
    }

    return 0;
}

void runBenchmark(string &detectorType, string &descriptorType, int &numKeypoints, int &numMatchedKeypoints, double &benchTime)
{
    if ( ( detectorType.compare("AKAZE") != 0 && descriptorType.compare("AKAZE") == 0 ) ||
         ( detectorType.compare("SIFT") == 0  && descriptorType.compare("ORB")   == 0 ) )
    {
        cerr << __FILE__ << "[" << __LINE__ << "] in function " << __FUNCTION__ << ": detector \"" << detectorType << "\" cannot be combined with descriptor \"" << descriptorType << "\"" << endl;
        numKeypoints = -1;
        numMatchedKeypoints = -1;
        benchTime = -1;
        return;
    }
 
    benchTime = 0.0;
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
        // remove front elements until only one element is left in the databuffer
        if( dataBuffer.size() == 2 )
        {
            dataBuffer.erase( dataBuffer.begin() );
        }
        else if ( dataBuffer.size() > 2 )
        {
            dataBuffer.erase( dataBuffer.begin(), dataBuffer.end() - 2 );
        }
        cout << "-------------------- #images in buffer: " << dataBuffer.size() << " --------------------- " << endl;

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
//         string detectorType = "SIFT"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        benchTime += detKeypointsModern( keypoints, imgGray, detectorType, false ); // covers also the "old" / heritage types
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            auto kp = keypoints.begin();
            while( kp != keypoints.end() )
            {
                // virtual rectangle: vehicleRect enlarged by 
                if( vehicleRect.contains( kp->pt ) )
                {
                    kp++;
                }
                else
                {
                    keypoints.erase( kp );
                }
            }
        }

        //// EOF STUDENT ASSIGNMENT

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
        cout << "#2 : DETECT KEYPOINTS done" << endl;
        numKeypoints += keypoints.size();

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
//         string descriptorType = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        benchTime += descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType2 = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

            cout << "INFO: " << __LINE__ << ": detectorType=\"" << detectorType << "\",   descriptorType=\"" << descriptorType << "\"" << endl;
            if( ( detectorType.compare("SHITOMASI") == 0 && descriptorType.compare("BRISK") == 0 ) ||
                ( detectorType.compare("SHITOMASI") == 0 && descriptorType.compare("BRIEF") == 0 ) ||
                ( detectorType.compare("SHITOMASI") == 0 && descriptorType.compare("FREAK") == 0 )  ||
                ( detectorType.compare("SHITOMASI") == 0 && descriptorType.compare("ORB") == 0 ) )
            {
                cout << "INFO: " << __LINE__ << ": " << endl;
                descriptorType2 = "DES_HOG";
            }
//             descriptorType

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            double matchTime = matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType2, matcherType, selectorType);
            benchTime += matchTime;

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            numMatchedKeypoints += matches.size();

            // visualize matches between current and previous image
            bVis = false;
            
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images
}
