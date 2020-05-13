#include <numeric>
#include <functional>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "matching2D.hpp"

using namespace std;

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
/// taken from here: https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
/// helper function to convert a data type of cv::Mat to a string for debugging purpose

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
/// mapping of detector name to its function to do the detection of keypoints

const std::map<std::string, std::function<double(std::vector<cv::KeyPoint>&, cv::Mat&, bool)>> KeypointFnMap =
    {
        {"HARRIS",    std::bind( detKeypointsHarris, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 )  },
        {"SHITOMASI", std::bind( detKeypointsShiTomasi, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 )  },
        {"FAST",      std::bind( detKeypointsFast, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 )  },
        {"BRISK",     std::bind( detKeypointsBrisk, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 )  },
        {"ORB",       std::bind( detKeypointsOrb, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 )  },
        {"AKAZE",     std::bind( detKeypointsAkaze, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 )  },
        {"SIFT",      std::bind( detKeypointsSift, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 )  }
    };

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Find best matches for keypoints in two camera images based on several matching methods
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    double deltaTime = 0.0;
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

//         cerr << "CHECKPOINT: " << __FILE__ << ", " << __LINE__ << " in function " << __FUNCTION__ << endl;
    if (descSource.type() != CV_32F)
    { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
        descSource.convertTo(descSource, CV_32F);
    }
    if (descRef.type() != CV_32F)
    { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
        descRef.convertTo(descRef, CV_32F);
    }

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout << "BF matching cross-check=" << crossCheck << ", ";
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
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
        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        deltaTime = t;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        int kLevel = ( matcherType.compare("MAT_FLANN") == 0 ) ? 2 : 1;
        double t = (double)cv::getTickCount();
        
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

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        deltaTime = t;
    }
    else
    {
        cerr << endl << "Wrong parameter for selectorType" << endl;
    }
    
// ------------------------------------------------------------------------------------------------------------------------------------------------
    //-- Filter matches using the Lowe's ratio test
    /*const float ratio_thresh = 0.8f;
    auto currentMatch = matches.begin();
    while( currentMatch != matches.end() )
    {
        if ((*currentMatch)[0].distance < ratio_thresh * (*currentMatch)[1].distance)
        {
            currentMatch++;
        }
        else
        {
            matches.erase( currentMatch );
        }
    }*/
// ------------------------------------------------------------------------------------------------------------------------------------------------
    for (size_t i = 0; i < matches.size(); i++)
    {
        if( matches[i].imgIdx >= matches.size() ) cout << "Suspicous entry " << i << " is pointing to entry " << matches[i].imgIdx << " which is out of bounds (" << matches.size() << ")" << endl;
        if( matches[i].queryIdx >= matches.size() ) cout << "Suspicous entry " << i << " is pointing to entry " << matches[i].queryIdx << " which is out of bounds (" << matches.size() << ")" << endl;
    }
// ------------------------------------------------------------------------------------------------------------------------------------------------
    return deltaTime;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        cerr << "ERROR: " << __FILE__ << ", " << __LINE__ << " in function " << __FUNCTION__ << ": \"" << descriptorType << "\" is not a valid descriptor type" << endl;
        //...
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

void vis(string windowName, vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    vis( "Shi-Tomasi Corner Detector Results", keypoints, img, bVis );
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    double t = (double)cv::getTickCount();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    double min, max;
    float tolerance = 0.6; // consider all points within the highest 2% of the value range
    cv::minMaxLoc( dst, &min, &max);
    min = max - (max - min) * tolerance;

    std::mutex myMutex;
    dst.forEach< cv::Vec<float,1> >([min,max,&myMutex,&keypoints](cv::Vec<float,1> &p, const int * position) -> void {
        if( p[0] >= min && p[0] <= max )
        {
            cv::KeyPoint kp( (float)position[1], (float)position[0], 4 + 4 * p[ 0 ] );
            {
                std::lock_guard<std::mutex> lk(myMutex);
                //std::cout << "Found keypoint @(" << position[0] << ", " << position[1] << ") with val = " << p[0] << std::endl;
                keypoints.push_back( kp );
            }
        }
    });
    std::cout << "Found key points: " << keypoints.size() << " with min=" << min << " and max=" << max << std::endl;

    std::sort (keypoints.begin(), keypoints.end(), []( const cv::KeyPoint &i, const cv::KeyPoint &j) -> bool { return ( i.size < j.size ); });

    vector<cv::KeyPoint>::iterator kp = keypoints.begin();
    while( kp != keypoints.end() )
    {
        vector<cv::KeyPoint>::iterator kp2 = kp;
        kp2++;
        while( kp2 != keypoints.end() )
        {
            if( ( kp->pt.x - kp2->pt.x ) * ( kp->pt.x - kp2->pt.x ) + ( kp->pt.y - kp2->pt.y ) * ( kp->pt.y - kp2->pt.y ) <= ( kp->size + kp2->size ) * ( kp->size + kp2->size ) )
            {
                keypoints.erase( kp2 );
            }
            else
            {
                kp2++;
            }
        }
        kp++;
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    // visualize results
    vis( "Harris Corner Detector Results", keypoints, img, bVis );
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

double detKeypointsFast(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    int blockSize = 6;       //  size of a block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints
    double qualityLevel = 0.01;                                   // minimal accepted quality of image corners
    double k = 0.04;

    // Initiate FAST object with default values
    int threshold = 30;                                                              // difference between intensity of the central pixel and pixels of a circle around this pixel
    bool bNMS = true;                                                                // perform non-maxima suppression on keypoints
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
    cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(threshold, bNMS, type);

    //fast->detect( img, keypoints );
    // find and draw the keypoints
    vector<cv::KeyPoint> kptsFast;
    vector<cv::Point2f> fastCorners;
    //----------------------------
    double t = (double)cv::getTickCount();
    fast->detect( img, kptsFast );
    cv::Mat fastMask(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    cv::drawKeypoints(fastMask, kptsFast, fastMask, cv::Scalar(1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    cv::goodFeaturesToTrack(img, fastCorners, maxCorners, qualityLevel, minDistance, fastMask, blockSize, false /*useHarris*/, k);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "FAST with n= " << fastCorners.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    keypoints.clear();
    for (auto it = fastCorners.begin(); it != fastCorners.end(); ++it)
    { // add fastCorners to result vector

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }

    //----------------------------
    // visualize results
    vis( "FAST Corner Detector Results", keypoints, img, bVis );
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

double detKeypointsBrisk(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BRISK detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    //----------------------------
    // visualize results
    vis( "BRISK Corner Detector Results", keypoints, img, bVis );
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

double detKeypointsOrb(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "ORB detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    //----------------------------
    // visualize results
    vis( "ORB Corner Detector Results", keypoints, img, bVis );
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

double detKeypointsAkaze(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "AKAZE detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    //----------------------------
    // visualize results
    vis( "AKAZE Corner Detector Results", keypoints, img, bVis );
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

double detKeypointsSift(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "SIFT detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    //----------------------------
    // visualize results
    vis( "SIFT Corner Detector Results", keypoints, img, bVis );
    return t;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    auto detectorFn = KeypointFnMap.find( detectorType );   // look up function based on a string->fn mapping declared in matching*
    double t = 0.0;;
    if( detectorFn != KeypointFnMap.end() )
    {
        t = detectorFn->second( keypoints, img, bVis );
    }
    else
    {
        cerr << "ERROR: Detector \"" << detectorType << "\" not found. Available types: ";
        for( auto key = KeypointFnMap.begin(); key != KeypointFnMap.end(); key++ )
        {
            cerr << key->first;
            cerr << ", ";
        }
        cerr << endl;
    }
    return t;
}
