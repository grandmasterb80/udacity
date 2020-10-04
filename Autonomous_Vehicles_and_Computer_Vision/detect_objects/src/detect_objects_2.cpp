#include <iostream>
#include <numeric>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

#include "dataStructures.h"

using namespace std;

/*
 * 1. Right out of the box on some pictures taken during vacation, the network delivers extrem good results. Traffic lights, trains, buses, dogs, people,
 *    cars have been quite reliably detected (even if confidence level was low for some cases). I noted that smaller objects, especially tall ones seem
 *    to be detected more often as "person".
 * 2. The blopsize must be a multiple of 32.
 *    The classification result depends on the blob size. A dog that played in the water was detected fine with the default settings (although the image
 *    showed only his back). With a smaller (also with a larger) blop size, the network deteced a boat.
 *    For the example image from the course, the result also depends on the blop size. If the person was detected, the classification was correct, but it
 *    could happen that the person or the car on the board were not detected at all (for blop size 64x64). Increase the blop size by the smallest increment
 *    (which is 32), reduced the confidence on the board.
 * 3. A lower threshold for confidence will determine how many objects are considered at all for further tests. Lower threshold means more classifications / "boxes2
 *    are going to be considered for further proceesing which can be a lot in real traffic.
 * 
 */

void detectObjects2(const string &filename_image)
{
    // load image from file
//     cv::Mat imgSrc = cv::imread( filename_image );
//     cv::Mat img;
//     resize(imgSrc, img, cv::Size(), 0.25, 0.25, cv::INTER_CUBIC);
    cv::Mat img = cv::imread( filename_image );

    // load class names from file
    string yoloBasePath = "../dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights"; 

    vector<string> classes;
    ifstream ifs(yoloClassesFile.c_str());
    string line;
    if( !ifs.is_open() )
    {
        cerr << "ERROR: " << __FILE__ << ", " << __LINE__ << " in function " << __FUNCTION__ << " could not open file" << endl;
        return;
    }
    while (getline(ifs, line)) classes.push_back(line);
    
    // load neural network
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
//     net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);

    double t = (double)cv::getTickCount();
    // generate 4D blob from input image
    cv::Mat blob;
    double scalefactor = 1/255.0;
    cv::Size size = cv::Size(416, 416);
//     cv::Size size = cv::Size(416, 416);
    cv::Scalar mean = cv::Scalar(0,0,0);
    bool swapRB = false;
    bool crop = false;
    cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BENCHMARK: blobFromImage in " << 1000 * t / 1.0 << " ms" << endl;
    t = (double)cv::getTickCount();

    // Get names of output layers
    vector<cv::String> names;
    vector<int> outLayers = net.getUnconnectedOutLayers(); // get indices of output layers, i.e. layers with unconnected outputs
    vector<cv::String> layersNames = net.getLayerNames(); // get names of all layers in the network
    
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i) // Get the names of the output layers in names
    {
        names[i] = layersNames[outLayers[i] - 1];
    }

    // invoke forward propagation through network
    vector<cv::Mat> netOutput;
    net.setInput(blob);
    net.forward(netOutput, names);

    // Scan through all bounding boxes and keep only the ones with high confidence
    float confThreshold = 0.2;
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect> boxes;
    for (size_t i = 0; i < netOutput.size(); ++i)
    {
        float* data = (float*)netOutput[i].data;
        for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols)
        {
            cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
            cv::Point classId;
            double confidence;
            
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
            if (confidence > confThreshold)
            {
                cv::Rect box; int cx, cy;
                cx = (int)(data[0] * img.cols);
                cy = (int)(data[1] * img.rows);
                box.width = (int)(data[2] * img.cols);
                box.height = (int)(data[3] * img.rows);
                box.x = cx - box.width/2; // left
                box.y = cy - box.height/2; // top
                
                boxes.push_back(box);
                classIds.push_back(classId.x);
                confidences.push_back((float)confidence);
            }
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BENCHMARK: detected " << netOutput.size() << " potential objects in " << 1000 * t / 1.0 << " ms" << endl;
    t = (double)cv::getTickCount();

    // perform non-maxima suppression
    float nmsThreshold = 0.5;  // Non-maximum suppression threshold
    vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    std::vector<BoundingBox> bBoxes;
    for (auto it = indices.begin(); it != indices.end(); ++it)
    {
        BoundingBox bBox;
        bBox.roi = boxes[*it];
        bBox.classID = classIds[*it];
        bBox.confidence = confidences[*it];
        bBox.boxID = (int)bBoxes.size(); // zero-based unique identifier for this bounding box
        
        bBoxes.push_back(bBox);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BENCHMARK: classification of " << indices.size() << " objects in " << 1000 * t / 1.0 << " ms" << endl;
    
    
    // show results
    cv::Mat visImg = img.clone();
    for (auto it = bBoxes.begin(); it != bBoxes.end(); ++it)
    {
        // Draw rectangle displaying the bounding box
        int top, left, width, height;
        top = (*it).roi.y;
        left = (*it).roi.x;
        width = (*it).roi.width;
        height = (*it).roi.height;
        cv::rectangle(visImg, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);

        string label = cv::format("%.2f", (*it).confidence);
        label = classes[((*it).classID)] + ":" + label;

        // Display label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        rectangle(visImg, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
    }

    string windowName = "Object classification";
    cv::namedWindow( windowName, 1 );
    cv::imshow( windowName, visImg );
    cv::waitKey(0); // wait for key to be pressed
}

int main(int argn, char** argv)
{
    string filename;
    if( argn >= 2 )
    {
        filename = argv[1];
    }
    else
    {
        filename = "../images/s_thrun.jpg";
//        filename = "/mnt/windows_BilderBackup/Bilder_Filme/2012 Liverpool/_MG_4075.JPG";
//        filename = "/mnt/windows_BilderBackup/Bilder_Filme/2012 Liverpool/_MG_4118.JPG";
//        filename = "/mnt/windows_BilderBackup/Bilder_Filme/2012 Liverpool/_MG_4119.JPG";
//        filename = "/mnt/windows_BilderBackup/Bilder_Filme/2012 Liverpool/_MG_4120.JPG";
    }
    detectObjects2( filename );
}
