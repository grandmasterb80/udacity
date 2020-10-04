#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <mutex>
#include <thread>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    vector<cv::KeyPoint> myResult;
    double min, max;
    float tolerance = 0.6; // consider all points within the highest 2% of the value range
    cv::minMaxLoc( dst, &min, &max);
    min = max - (max - min) * tolerance;

    std::mutex myMutex;
    dst.forEach< cv::Vec<float,1> >([min,max,&myMutex,&myResult](cv::Vec<float,1> &p, const int * position) -> void {
        if( p[0] >= min && p[0] <= max )
        {
            cv::KeyPoint kp( (float)position[1], (float)position[0], 4 + 4 * p[ 0 ] );
            {
                std::lock_guard<std::mutex> lk(myMutex);
                //std::cout << "Found keypoint @(" << position[0] << ", " << position[1] << ") with val = " << p[0] << std::endl;
                myResult.push_back( kp );
            }
        }
    });
    std::cout << "Found key points: " << myResult.size() << " with min=" << min << " and max=" << max << std::endl;

    std::sort (myResult.begin(), myResult.end(), []( const cv::KeyPoint &i, const cv::KeyPoint &j) -> bool { return ( i.size < j.size ); });

    vector<cv::KeyPoint>::iterator kp = myResult.begin();
    while( kp != myResult.end() )
    {
        vector<cv::KeyPoint>::iterator kp2 = kp;
        kp2++;
        while( kp2 != myResult.end() )
        {
            if( ( kp->pt.x - kp2->pt.x ) * ( kp->pt.x - kp2->pt.x ) + ( kp->pt.y - kp2->pt.y ) * ( kp->pt.y - kp2->pt.y ) <= ( kp->size + kp2->size ) * ( kp->size + kp2->size ) )
            {
                myResult.erase( kp2 );
            }
            else
            {
                kp2++;
            }
        }
        kp++;
    }

    cv::Mat myOutput;
    drawKeypoints(dst_norm_scaled, myResult, myOutput, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // ------------------------------------------------------------------------------------------------
    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, myOutput);
    cv::waitKey(0);
}

int main()
{
    cornernessHarris();
}
