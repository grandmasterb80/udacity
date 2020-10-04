#include <iostream>
#include <numeric>
#include <list>
#include <tuple>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "structIO.hpp"

using namespace std;

void matchDescriptors(cv::Mat &imgSource, cv::Mat &imgRef, vector<cv::KeyPoint> &kPtsSource, vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      vector<cv::DMatch> &matches, string descriptorType, string matcherType, string selectorType, int dataSet)
{

    cout << "Calling \"matchDescriptors\" with data set: " << dataSet << " and matcherType = \"" << matcherType << "\"" << ", descriptorType = \"" << descriptorType << "\"" << ", selectorType = \"" << selectorType << "\"" << endl;

    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (descSource.type() != CV_32F)
    { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);
    }

    if (matcherType.compare("MAT_BF") == 0)
    {
//         cout << __FILE__ << ", " << __LINE__ << ": xxx" << endl;
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout << "BF matching cross-check=" << crossCheck << ", ";
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
//         cout << __FILE__ << ", " << __LINE__ << ": xxx" << endl;
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        //... TODO : implement FLANN matching
        matcher = cv::DescriptorMatcher::create( cv::DescriptorMatcher::FLANNBASED );
        cout << "FLANN matching";
    }
    else
    {
        cerr << endl << "Wrong parameter for matcherType" << endl;
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
//         cout << __FILE__ << ", " << __LINE__ << ": xxx" << endl;
        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
//         cout << __FILE__ << ", " << __LINE__ << ": xxx" << endl;
        // TODO : implement k-nearest-neighbor matching
        int kLevel = ( matcherType.compare("MAT_FLANN") == 0 ) ? 2 : 1;
        double t = (double)cv::getTickCount();
        // ----------------------------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------------------------
        std::vector< std::vector< cv::DMatch > > knn_matches;        

        // Since SURF is a floating-point descriptor NORM_L2 is used
        matcher->knnMatch( descSource, descRef, knn_matches, kLevel );
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.8f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back( knn_matches[i][0] );
            }
        }
        //-- Draw matches
        // ----------------------------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------------------------
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        // TODO : filter matches using descriptor distance ratio test
    }
    else
    {
        cerr << endl << "Wrong parameter for selectorType" << endl;
    }
/*
    for (size_t i = 0; i < matches.size(); i++)
    {
        if( matches[i].imgIdx >= matches.size() ) cout << "Suspicous entry " << i << " is pointing to entry " << matches[i].imgIdx << " which is out of bounds (" << matches.size() << ")" << endl;
        if( matches[i].queryIdx >= matches.size() ) cout << "Suspicous entry " << i << " is pointing to entry " << matches[i].queryIdx << " which is out of bounds (" << matches.size() << ")" << endl;
    }
*/    
//     cout << __FILE__ << ", " << __LINE__ << ": xxx" << endl;
    // visualize results
    cv::Mat matchImg = imgRef.clone();
    cv::drawMatches(imgSource, kPtsSource, imgRef, kPtsRef, matches,
                    matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    string windowName = "Matching keypoints between two camera images (best 50)";
    cv::namedWindow(windowName, 7);
    cv::imshow(windowName, matchImg);
    cv::waitKey(0);
}

int main(int argn, char** argc)
{
//     CommandLineParser parser( argc, argv, keys );
    cout << endl;
    cout << endl;
//     cout << "Usage:" << endl;
//     cout << argc[0] << " [--matcher={MAT_BF,MAT_FLANN}] --[descriptor={DES_BINARY,X}] [--selector={SEL_NN,SEL_KN}]" << endl;
//     cout << "    --matcher={MAT_BF,MAT_FLANN} --descriptor={DES_BINARY,X}] [selector={SEL_NN,SEL_KN}]" << endl;
//     cout << "    --descriptor={DES_BINARY,X}] [selector={SEL_NN,SEL_KN}]" << endl;
//     cout << "    --selector={SEL_NN,SEL_KN}" << endl;
//     cout << endl;
//     cout << endl;
//     cout << __FILE__ << ", " << __LINE__ << ": Reading image \"../images/img1gray.png\"" << endl;
//     cout << __FILE__ << ", " << __LINE__ << ": Reading image \"../images/img2gray.png\"" << endl;

    list< tuple< int, string, string, string > > combinations = {
        std::make_tuple(0, "MAT_BF",    "DES_HOG", "SEL_NN"),
        std::make_tuple(0, "MAT_BF",    "DES_HOG", "SEL_KNN"),
        std::make_tuple(1,  "MAT_BF",    "DES_HOG", "SEL_NN"),
        std::make_tuple(1,  "MAT_BF",    "DES_HOG", "SEL_KNN"),

        std::make_tuple(1,  "MAT_BF",    "DES_HOG", "SEL_NN"),
        std::make_tuple(1,  "MAT_FLANN", "DES_HOG", "SEL_NN"),
        std::make_tuple(2,  "MAT_BF",    "DES_HOG", "SEL_NN"),
        std::make_tuple(2,  "MAT_FLANN", "DES_HOG", "SEL_NN"),

        std::make_tuple(1,  "MAT_BF",    "DES_HOG", "SEL_KNN"),
        std::make_tuple(1,  "MAT_FLANN", "DES_HOG", "SEL_KNN"),
        std::make_tuple(2,  "MAT_BF",    "DES_HOG", "SEL_KNN"),
        std::make_tuple(2,  "MAT_FLANN", "DES_HOG", "SEL_KNN")
    };

    for( list< tuple< int, string, string, string > >::iterator comb = combinations.begin(); comb != combinations.end(); comb++ )
    {
        vector<cv::KeyPoint> kptsSource, kptsRef; 
        cv::Mat descSource, descRef; 

        int dataSet; // = std::get<0>(*comb);
        string matcherType;
        string descriptorType;
        string selectorType;

        std::tie( dataSet, matcherType, descriptorType, selectorType ) = *comb;

//         cout << "Loading...";
        cv::Mat imgSource = cv::imread("../images/img1gray.png");
        cv::Mat imgRef = cv::imread("../images/img2gray.png");

        switch( dataSet )
        {
            case 0:
                readKeypoints("../dat/C35A5_KptsSource_BRISK_large.dat", kptsSource);
                readKeypoints("../dat/C35A5_KptsRef_BRISK_large.dat", kptsRef);
                readDescriptors("../dat/C35A5_DescSource_BRISK_large.dat", descSource);
                readDescriptors("../dat/C35A5_DescRef_BRISK_large.dat", descRef);
                break;
            case 1:
                readKeypoints("../dat/C35A5_KptsSource_BRISK_small.dat", kptsSource);
                readKeypoints("../dat/C35A5_KptsRef_BRISK_small.dat", kptsRef);
                readDescriptors("../dat/C35A5_DescSource_BRISK_small.dat", descSource);
                readDescriptors("../dat/C35A5_DescRef_BRISK_small.dat", descRef);
                break;
            case 2:
                readKeypoints("../dat/C35A5_KptsSource_SIFT.dat", kptsSource);
                readKeypoints("../dat/C35A5_KptsRef_SIFT.dat", kptsRef);
                readDescriptors("../dat/C35A5_DescSource_SIFT.dat", descSource);
                readDescriptors("../dat/C35A5_DescRef_SIFT.dat", descRef);
                if (descSource.type() != CV_32F)
                { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
                    descSource.convertTo(descSource, CV_32F);
                    descRef.convertTo(descRef, CV_32F);
                }
                break;
            default:    
                cerr << "Unknown data set request" << endl;
                break;
        }

        vector<cv::DMatch> matches;
        matchDescriptors(imgSource, imgRef, kptsSource, kptsRef, descSource, descRef, matches, descriptorType, matcherType, selectorType, dataSet);
    }

/*
    -------------------------------------------------------------------------------
    string matcherType = "MAT_BF";
    string descriptorType = "DES_BINARY";
    string selectorType = "SEL_NN"; 
    BF matching cross-check=0 (NN) with n=2896 matches in 62.8159 ms
    -------------------------------------------------------------------------------
    string matcherType = "MAT_BF";
    string descriptorType = "";
    string selectorType = "SEL_NN"; 
    BF matching cross-check=0 (NN) with n=2896 matches in 30.0812 ms
    -------------------------------------------------------------------------------
    string matcherType = "MAT_BF";
    string descriptorType = "DES_BINARY";
    string selectorType = "SEL_NN"; 
    BF matching cross-check=1 (NN) with n=1705 matches in 23.6244 ms
    -------------------------------------------------------------------------------

    
    string matcherType = "MAT_FLANN";
    string descriptorType = "DES_BINARY";
    string selectorType = "SEL_NN"; 
    FLANN matching (NN) with n=2896 matches in 26.8072 ms
    -------------------------------------------------------------------------------
    string matcherType = "MAT_FLANN";
    string descriptorType = "DES_BINARY";
    string selectorType = "SEL_KNN"; 
    FLANN matching (NN) with n=1177 matches in 27.3481 ms
    -------------------------------------------------------------------------------
Calling "matchDescriptors" with small data set and matcherType = "MAT_BF", descriptorType = "DES_BINARY", selectorType = "SEL_NN"     BF    matching cross-check=0,  (NN) with n=100 matches  in 1.00857 ms
Calling "matchDescriptors" with small data set and matcherType = "MAT_BF", descriptorType = "DES_BINARY", selectorType = "SEL_KNN"    BF    matching cross-check=0,  (NN) with n=1 matches    in 1.3243 ms
Calling "matchDescriptors" with small data set and matcherType = "MAT_BF", descriptorType = "", selectorType = "SEL_NN"               BF    matching cross-check=0,  (NN) with n=100 matches  in 2.93332 ms
Calling "matchDescriptors" with small data set and matcherType = "MAT_BF", descriptorType = "", selectorType = "SEL_KNN"              BF    matching cross-check=0,  (NN) with n=1 matches    in 0.271545 ms
Calling "matchDescriptors" with small data set and matcherType = "MAT_FLANN", descriptorType = "DES_BINARY", selectorType = "SEL_NN"  FLANN matching (NN)                 with n=100 matches  in 0.849446 ms
Calling "matchDescriptors" with small data set and matcherType = "MAT_FLANN", descriptorType = "DES_BINARY", selectorType = "SEL_KNN" FLANN matching (NN)                 with n=44 matches   in 0.833565 ms
Calling "matchDescriptors" with small data set and matcherType = "MAT_FLANN", descriptorType = "", selectorType = "SEL_NN"            FLANN matching (NN)                 with n=100 matches  in 1.02251 ms
Calling "matchDescriptors" with small data set and matcherType = "MAT_FLANN", descriptorType = "", selectorType = "SEL_KNN"           FLANN matching (NN)                 with n=44 matches   in 1.0116 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_BF", descriptorType = "DES_BINARY", selectorType = "SEL_NN"     BF    matching cross-check=0,  (NN) with n=2896 matches in 15.4574 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_BF", descriptorType = "DES_BINARY", selectorType = "SEL_KNN"    BF    matching cross-check=0,  (NN) with n=1 matches    in 16.6797 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_BF", descriptorType = "", selectorType = "SEL_NN"               BF    matching cross-check=0,  (NN) with n=2896 matches in 29.5635 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_BF", descriptorType = "", selectorType = "SEL_KNN"              BF    matching cross-check=0,  (NN) with n=1 matches    in 28.979 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_FLANN", descriptorType = "DES_BINARY", selectorType = "SEL_NN"  FLANN matching (NN)                 with n=2896 matches in 27.2596 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_FLANN", descriptorType = "DES_BINARY", selectorType = "SEL_KNN" FLANN matching (NN)                 with n=1186 matches in 27.9829 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_FLANN", descriptorType = "", selectorType = "SEL_NN"            FLANN matching (NN)                 with n=2896 matches in 27.493 ms
Calling "matchDescriptors" with large data set and matcherType = "MAT_FLANN", descriptorType = "", selectorType = "SEL_KNN"           FLANN matching (NN)                 with n=1168 matches in 28.2349 ms






Calling "matchDescriptors" with data set: 0 and matcherType = "MAT_BF", descriptorType = "DES_HOG", selectorType = "SEL_NN"         BF matching cross-check=0,  (NN) with n=2896 matches in 15.523 ms
Calling "matchDescriptors" with data set: 0 and matcherType = "MAT_BF", descriptorType = "DES_HOG", selectorType = "SEL_KNN"        BF matching cross-check=0,  (NN) with n=1 matches in 13.6261 ms
Calling "matchDescriptors" with data set: 1 and matcherType = "MAT_BF", descriptorType = "DES_HOG", selectorType = "SEL_NN"         BF matching cross-check=0,  (NN) with n=100 matches in 1.21725 ms
Calling "matchDescriptors" with data set: 1 and matcherType = "MAT_BF", descriptorType = "DES_HOG", selectorType = "SEL_KNN"        BF matching cross-check=0,  (NN) with n=1 matches in 0.158452 ms

Calling "matchDescriptors" with data set: 1 and matcherType = "MAT_BF",    descriptorType = "DES_HOG", selectorType = "SEL_NN"      BF matching cross-check=0,  (NN) with n=100 matches in 0.11219 ms
Calling "matchDescriptors" with data set: 1 and matcherType = "MAT_FLANN", descriptorType = "DES_HOG", selectorType = "SEL_NN"      FLANN matching              (NN) with n=100 matches in 0.84425 ms
Calling "matchDescriptors" with data set: 1 and matcherType = "MAT_BF",    descriptorType = "DES_HOG", selectorType = "SEL_KNN"     BF matching cross-check=0,  (NN) with n=0 matches in 0.121924 ms
Calling "matchDescriptors" with data set: 1 and matcherType = "MAT_FLANN", descriptorType = "DES_HOG", selectorType = "SEL_KNN"     FLANN matching              (NN) with n=45 matches in 1.33006 ms

Calling "matchDescriptors" with data set: 2 and matcherType = "MAT_BF",    descriptorType = "DES_HOG", selectorType = "SEL_NN"      BF matching cross-check=0,  (NN) with n=1890 matches in 11.0526 ms
Calling "matchDescriptors" with data set: 2 and matcherType = "MAT_FLANN", descriptorType = "DES_HOG", selectorType = "SEL_NN"      FLANN matching              (NN) with n=1890 matches in 21.1011 ms
Calling "matchDescriptors" with data set: 2 and matcherType = "MAT_BF",    descriptorType = "DES_HOG", selectorType = "SEL_KNN"     BF matching cross-check=0,  (NN) with n=0 matches in 10.3127 ms
Calling "matchDescriptors" with data set: 2 and matcherType = "MAT_FLANN", descriptorType = "DES_HOG", selectorType = "SEL_KNN"     FLANN matching              (NN) with n=1036 matches in 21.5971 ms

*/
}
