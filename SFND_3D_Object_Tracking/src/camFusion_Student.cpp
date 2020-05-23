
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

extern bool DOWNSIZE_VIS;

using namespace std;

// ------------------------------------------------------------------------------------------------------------------------------------------------

TTCMethod ttcMethodLidar = TTCAverage10;
TTCMethod ttcMethodCam   = TTCAverage10;

std::string ttcMethodStr(TTCMethod m)
{
	switch(m)
	{
		case TTCMedian:
			return std::string("Median");
		case TTCAverage10:
			return std::string("Avg10");
		case TTCAverage10_First10:
			return std::string("Avg10Cl10");
		case TTCAverageSmallestError:
			return std::string("AvgMinErr");
		default:
			return std::string("Unknown");
	}
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    double t = (double)cv::getTickCount();
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "DONE: " << __FUNCTION__ << " in " << 1000 * t / 1.0 << " ms" << endl;
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    if( DOWNSIZE_VIS ) resize(topviewImg, topviewImg, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &prevBoundingBox, BoundingBox &currBoundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    double t = (double)cv::getTickCount();
    const double tolerance = 0.2;
    double distanceMean = 0.0;
    int    distanceMeanElements = 0;
    std::vector<int> bbMatches; // indicies to matches of interest

    currBoundingBox.keypoints.clear();

    for( cv::DMatch &match : kptMatches )
    {
        int queryIdx = match.queryIdx; // source
        int trainIdx = match.trainIdx; // target

        if( ! prevBoundingBox.roi.contains( kptsPrev[ queryIdx ].pt ) )  continue;
        if( ! currBoundingBox.roi.contains( kptsCurr[ trainIdx ].pt ) )  continue;

        distanceMean += match.distance;
        distanceMeanElements++;
    }
    distanceMean /= distanceMeanElements;

    // second run: new add the elements to the bounding box that we are interested in
    for( cv::DMatch &match : kptMatches )
    //for( int matchIdx = 0; matchIdx < boundingBox.kptMatches.size(); matchIdx++ )
    {
        int queryIdx = match.queryIdx; // source
        int trainIdx = match.trainIdx; // target

        if( ! prevBoundingBox.roi.contains( kptsPrev[ queryIdx ].pt ) )  continue;
        if( ! currBoundingBox.roi.contains( kptsCurr[ trainIdx ].pt ) )  continue;

        if( match.distance <= distanceMean * (1.0 - tolerance) ||
            match.distance >= distanceMean * (1.0 + tolerance) ) continue;

        currBoundingBox.kptMatches.push_back( match );
        currBoundingBox.keypoints.push_back( kptsCurr[ trainIdx ] );
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "DONE: " << __FUNCTION__ << " in " << 1000 * t / 1.0 << " ms" << endl;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double t = (double)cv::getTickCount();
    double deltaTime = 1.0 / frameRate;

    // ...
    // sort matches according to the points based on y value (e.g. left to right)
    // take the outside matches
    auto matchSort = [&]( cv::DMatch& a, cv::DMatch& b )-> bool {
        return ( kptsCurr[ a.trainIdx ].pt.y ) < kptsCurr[ b.trainIdx ].pt.y;
    };

    std::sort( kptMatches.begin(), kptMatches.end(), matchSort );

	int idx1 = 0;
	int idx2 = kptMatches.size() - 1;

    int numMeasurements = 30; // max # measurements 
    std::vector<double> TTCs;
    while( numMeasurements > 0 && idx1 < idx2 )
    {
        int prevIdx1 = kptMatches[ idx1 ].queryIdx;      // source
        int currIdx1 = kptMatches[ idx1 ].trainIdx;      // target
        int prevIdx2 = kptMatches[ idx2 ].queryIdx;      // source
        int currIdx2 = kptMatches[ idx2 ].trainIdx;      // target
        //~ cout << __LINE__ << ": prevIdx1=" << prevIdx1 << ", currIdx1=" << currIdx1 << ", prevIdx2" << prevIdx2 << ", currIdx2" << currIdx2 << ", kptsPrev.size()=" << kptsPrev.size() << ", kptsCurr.size()=" << kptsCurr.size() << endl;
        assert( prevIdx1 >= 0 );
        assert( prevIdx1 < kptsPrev.size() );
        assert( prevIdx2 >= 0 );
        assert( prevIdx2 < kptsPrev.size() );
        assert( currIdx1 >= 0 );
        assert( currIdx1 < kptsCurr.size() );
        assert( currIdx2 >= 0 );
        assert( currIdx2 < kptsCurr.size() );

        cv::Point2f prevCon = kptsPrev[ prevIdx1 ].pt - kptsPrev[ prevIdx2 ].pt;
        cv::Point2f currCon = kptsCurr[ currIdx1 ].pt - kptsCurr[ currIdx2 ].pt;
        double h_prev = sqrt( prevCon.dot( prevCon ) );
        double h_curr = sqrt( currCon.dot( currCon ) );

        TTC = - deltaTime / ( 1 - h_curr / h_prev );
        if( fabs( h_curr - h_prev ) > 0.01 * deltaTime )
        {
            TTCs.push_back( TTC );
        }

        // cout << "TTC_cam: h_prev=" << h_prev << "(), h_curr=" << h_curr << "(), deltaTime=" << deltaTime << endl;
        // cout << "TTC_cam: TTC=" << TTC << endl;

        //for (size_t i = 0; i < nMarkers; ++i)
        if(visImg!=nullptr)
        {
            cv::line(*visImg, kptsPrev[ prevIdx1 ].pt, kptsPrev[ prevIdx2 ].pt, cv::Scalar( 255,   0,   0 ) );
            cv::line(*visImg, kptsCurr[ currIdx1 ].pt, kptsCurr[ currIdx2 ].pt, cv::Scalar(   0,   0, 255 ) );
        }
        else
        {
            cout << "*** no visualization available ***" << endl;
        }
        idx1++;
        idx2--;
        numMeasurements--;
    }

    if( TTCs.size() > 0 )
    {
        std::sort( TTCs.begin(), TTCs.end() );
        int TTCsSize = TTCs.size();
		switch( ttcMethodCam )
		{
			case TTCMedian:
			{
				TTC = TTCs[ TTCsSize / 2 ];
				break;
			}
			case TTCAverage10:
			{
				TTCs.erase( TTCs.begin(), TTCs.begin() + TTCsSize / 3 );
				TTCs.erase( TTCs.end() - 1 - TTCsSize / 3, TTCs.end() - 1 );
				TTC = std::accumulate( TTCs.begin(), TTCs.end(), 0.0 ) / TTCsSize;
				break;
			}
			case TTCAverage10_First10:
			{
				TTCs.erase( TTCs.begin(), TTCs.begin() + TTCsSize / 3 );
				TTC = std::accumulate( TTCs.begin(), TTCs.begin() + TTCsSize / 3, 0.0 ) / ( TTCsSize / 3);
				break;
			}
			case TTCAverageSmallestError:
			{
				double Xrange = 0.3;
				double tolerance = 0.05;
				double tolSquared = tolerance * tolerance;

				int currN = TTCs.size();
				TTC = std::accumulate( TTCs.begin(), TTCs.end(), 0.0 );

				int k = 0;
				int l = currN - 1;
				while( k < l )
				{
					double errorK = 0.0;
					double errorL = 0.0;
					double currDistI = ( TTC - TTCs[ k ] ) / ( currN - 1 );
					double currDistJ = ( TTC - TTCs[ l ] ) / ( currN - 1 );
					for(int sub = k + 1; sub < l - 1; sub++)
					{
						double eK = TTCs[ sub ] - currDistI; eK *= eK;
						double eL = TTCs[ sub ] - currDistJ; eL *= eL;
						errorK += eK;
						errorL += eL;
					}
					{
						double eK = TTCs[ k ] - currDistI; eK *= eK;
						double eL = TTCs[ l ] - currDistJ; eL *= eL;
						errorK += eK;
						errorL += eL;
					}
					errorK = errorK / (currN - 1);
					errorL = errorL / (currN - 1);

					if( errorK < tolSquared && errorL < tolSquared ) break;
					// remove the one causing the bigger error
					if( errorK > errorL )
					{
						TTC -= TTCs[ k ];
						k++;
					}
					else
					{
						TTC -= TTCs[ l ];
						l--;
					}
					currN--;
				}
				TTC /= currN;
				break;
			}
		}
    }
    else
    {
		cout << "TTC_cam: WARNING - not enough tracked points (" << kptMatches.size() << ")" << endl;
        TTC = 999.9;
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

	cout << "TTC_cam: TTC=" << TTC << endl;

    cout << "DONE: " << __FUNCTION__ << " in " << 1000 * t / 1.0 << " ms" << endl;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double t = (double)cv::getTickCount();
    double deltaTime = 1.0 / frameRate;
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // helper function: we only need the distances of the lidar points ==> we copy the values into a dedicated area and sort the area accordingly
    struct {
        bool operator()( LidarPoint& a, LidarPoint& b ) const
        {
            return a.x < b.x;
        }
    } lidarPointLess;

    struct {
        double operator()( double a, LidarPoint& b ) const
        {
            return a + b.x;
        }
    } lidarPointSumX;

    // sort points along distance
    std::sort( lidarPointsPrev.begin(), lidarPointsPrev.end(), lidarPointLess );
    std::sort( lidarPointsCurr.begin(), lidarPointsCurr.end(), lidarPointLess );
    double prevDist = 0.0;
    double currDist = 0.0;
    if( lidarPointsPrev.size() > 30 && lidarPointsCurr.size() > 30 )
    {
        switch( ttcMethodLidar )
        {
            case TTCMedian:
            {
                prevDist = lidarPointsPrev[ lidarPointsPrev.size() / 2 ].x;
                currDist = lidarPointsCurr[ lidarPointsCurr.size() / 2 ].x;
                break;
			}
            case TTCAverage10:
            {
                lidarPointsPrev.erase( lidarPointsPrev.begin(), lidarPointsPrev.begin() + 10 );
                lidarPointsPrev.erase( lidarPointsPrev.end() - 11, lidarPointsPrev.end() - 1 );

                lidarPointsCurr.erase( lidarPointsCurr.begin(), lidarPointsCurr.begin() + 10 );
                lidarPointsCurr.erase( lidarPointsCurr.end() - 11, lidarPointsCurr.end() - 1 );
                prevDist = std::accumulate( lidarPointsPrev.begin(), lidarPointsPrev.end(), 0.0, lidarPointSumX ) / lidarPointsPrev.size();
                currDist = std::accumulate( lidarPointsCurr.begin(), lidarPointsCurr.end(), 0.0, lidarPointSumX ) / lidarPointsCurr.size();
                break;
			}
            case TTCAverage10_First10:
            {
                lidarPointsPrev.erase( lidarPointsPrev.begin(), lidarPointsPrev.begin() + 10 );

                lidarPointsCurr.erase( lidarPointsCurr.begin(), lidarPointsCurr.begin() + 10 );
                prevDist = std::accumulate( lidarPointsPrev.begin(), lidarPointsPrev.begin() + 10, 0.0, lidarPointSumX ) / 10;
                currDist = std::accumulate( lidarPointsCurr.begin(), lidarPointsCurr.begin() + 10, 0.0, lidarPointSumX ) / 10;
                break;
			}
            case TTCAverageSmallestError:
            {
				double Xrange = 0.3;
				double tolerance = 0.05;
				double tolSquared = tolerance * tolerance;

				int prevN = lidarPointsPrev.size();
				int currN = lidarPointsCurr.size();
                prevDist = std::accumulate( lidarPointsPrev.begin(), lidarPointsPrev.end(), 0.0, lidarPointSumX );
                currDist = std::accumulate( lidarPointsCurr.begin(), lidarPointsCurr.end(), 0.0, lidarPointSumX );

				int i = 0;
				int j = prevN - 1;
				while( i < j )
				{
					double errorI = 0.0;
					double errorJ = 0.0;
					double prevDistI = ( prevDist - lidarPointsPrev[ i ].x ) / ( prevN - 1 );
					double prevDistJ = ( prevDist - lidarPointsPrev[ j ].x ) / ( prevN - 1 );
					for(int sub = i + 1; sub < j - 1; sub++)
					{
						double eI = lidarPointsPrev[ sub ].x - prevDistI; eI *= eI;
						double eJ = lidarPointsPrev[ sub ].x - prevDistJ; eJ *= eJ;
						errorI += eI;
						errorJ += eJ;
					}
					{
						double eI = lidarPointsPrev[ i ].x - prevDistI; eI *= eI;
						double eJ = lidarPointsPrev[ j ].x - prevDistJ; eJ *= eJ;
						errorI += eI;
						errorJ += eJ;
					}
					errorI = errorI / (prevN - 1);
					errorJ = errorJ / (prevN - 1);

					if( errorI < tolSquared && errorJ < tolSquared ) break;
					// remove the one causing the bigger error
					if( errorI > errorJ )
					{
						prevDist -= lidarPointsPrev[ i ].x;
						i++;
					}
					else
					{
						prevDist -= lidarPointsPrev[ j ].x;
						j--;
					}
					prevN--;
				}

				int k = 0;
				int l = currN - 1;
				while( k < l )
				{
					double errorK = 0.0;
					double errorL = 0.0;
					double currDistI = ( currDist - lidarPointsCurr[ k ].x ) / ( currN - 1 );
					double currDistJ = ( currDist - lidarPointsCurr[ l ].x ) / ( currN - 1 );
					for(int sub = k + 1; sub < l - 1; sub++)
					{
						double eK = lidarPointsCurr[ sub ].x - currDistI; eK *= eK;
						double eL = lidarPointsCurr[ sub ].x - currDistJ; eL *= eL;
						errorK += eK;
						errorL += eL;
					}
					{
						double eK = lidarPointsCurr[ k ].x - currDistI; eK *= eK;
						double eL = lidarPointsCurr[ l ].x - currDistJ; eL *= eL;
						errorK += eK;
						errorL += eL;
					}
					errorK = errorK / (currN - 1);
					errorL = errorL / (currN - 1);

					if( errorK < tolSquared && errorL < tolSquared ) break;
					// remove the one causing the bigger error
					if( errorK > errorL )
					{
						currDist -= lidarPointsCurr[ k ].x;
						k++;
					}
					else
					{
						currDist -= lidarPointsCurr[ l ].x;
						l--;
					}
					currN--;
				}
				prevDist /= prevN;
				currDist /= currN;
                break;
			}
        }
        // method: 
        // ---------------------------------------------------------------------------------------

        TTC = currDist * deltaTime / ( prevDist - currDist );

//         cout << "prev points idx = [" << i << ", " << j << "]     curr points idx = [" << k << ", " << l << "]" << endl;
//         cout << "TTC Lidar: prevN=" << prevN << "/" << lidarPointsPrev.size() << ", currN=" << currN << "/" << lidarPointsCurr.size() << ", prevDist=" << prevDist << ", currDist=" << currDist << " @fps=" << frameRate << endl;
        cout << "TTC_Lidar: prev closest = " << lidarPointsPrev[ 0 ].x << "    prev farest = " << lidarPointsPrev[ lidarPointsPrev.size() - 1 ].x << endl;
        cout << "TTC_Lidar: curr closest = " << lidarPointsCurr[ 0 ].x << "    curr farest = " << lidarPointsCurr[ lidarPointsCurr.size() - 1 ].x << endl;
        cout << "TTC_Lidar: TTC = " << TTC << endl;

    }
    else
    {
        TTC = 999.9;
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "DONE: " << __FUNCTION__ << " in " << 1000 * t / 1.0 << " ms" << endl;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

void clusterKeypointsWithROI(DataFrame &frame)
{
    double t = (double)cv::getTickCount();
    for( cv::KeyPoint& kp : frame.keypoints )
    {
        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator bb = frame.boundingBoxes.begin(); bb != frame.boundingBoxes.end(); ++bb)
        {
            if( bb->roi.contains( kp.pt ) ) 
            {
                enclosingBoxes.push_back( bb );
            }
        }
        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->keypoints.push_back( kp );
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "DONE: " << __FUNCTION__ << " in " << 1000 * t / 1.0 << " ms" << endl;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int matchCount[ prevFrame.boundingBoxes.size() ][ currFrame.boundingBoxes.size() ];
    double t = (double)cv::getTickCount();
    for(int i = 0; i < prevFrame.boundingBoxes.size(); i++)
    {
        for(int j = 0; j < currFrame.boundingBoxes.size(); j++)
        {
            matchCount[i][j] = ( prevFrame.boundingBoxes[ i ].roi & currFrame.boundingBoxes[ j ].roi ).area();
//             matchCount[i][j] = 0;
        }
    }
/*
    for( int matchIdx = 0; matchIdx < currFrame.kptMatches.size(); matchIdx++ )
    {
        int queryIdx = currFrame.kptMatches[ matchIdx ].queryIdx; // source
        int trainIdx = currFrame.kptMatches[ deceitmatchIdx ].trainIdx; // target

        vector<vector<BoundingBox>::iterator> prevBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator bb = prevFrame.boundingBoxes.begin(); bb != prevFrame.boundingBoxes.end(); ++bb)
        {
            if( bb->roi.contains( prevFrame.keypoints[ queryIdx ].pt ) ) 
            {
                prevBoxes.push_back( bb );
            }
        }

        vector<vector<BoundingBox>::iterator> currBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator bb = currFrame.boundingBoxes.begin(); bb != currFrame.boundingBoxes.end(); ++bb)
        {
            if( bb->roi.contains( currFrame.keypoints[ trainIdx ].pt ) ) 
            {
                currBoxes.push_back( bb );
                currFrame.boundingBoxes[ bb->boxID ].kptMatches.push_back( currFrame.kptMatches[ matchIdx ] );
            }
        }

        // find bounding box in prevFrame with this given keypoint
        for( vector<BoundingBox>::iterator prevBB : prevBoxes )
        {
            for( vector<BoundingBox>::iterator currBB : currBoxes )
            {
                matchCount[ prevBB->boxID ][ currBB->boxID ]++;
            }
        }
    }
*/
    for(int i = 0; i < prevFrame.boundingBoxes.size(); i++)
    {
//         cout << "i=" << i << ": {";
        int mc = 0;
        int matchBBIdx = -1;

        for(int j = 0; j < currFrame.boundingBoxes.size(); j++)
        {
//             if(j!=0) cout << ", ";
//             cout << matchCount[i][j];
            if( matchCount[i][j] > mc ) { matchBBIdx = j; mc = matchCount[i][j]; }
        }
//         cout << "}" << endl;
        if( matchBBIdx >= 0 )
        {
//             cout << __FILE__ << ", " << __LINE__ << ": prevFrame.boxID=" << prevFrame.boundingBoxes[ i ].boxID << "(idx=" << i << ") ---> currFrame.boxID=" << currFrame.boundingBoxes[ matchBBIdx ].boxID << "(idx=" << matchBBIdx << ")" << endl;
            bbBestMatches[ prevFrame.boundingBoxes[ i ].boxID ] = currFrame.boundingBoxes[ matchBBIdx ].boxID;
        }
        else
        {
            cout << "CHECKPOINT: " << __FILE__ << ", " << __LINE__ << " no match found" << endl;
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "DONE: " << __FUNCTION__ << " in " << 1000 * t / 1.0 << " ms" << endl;
}
