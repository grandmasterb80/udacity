// load image from file
cv::Mat img = cv::imread("./images/img1.png");

// load class names from file
string yoloBasePath = "./dat/yolo/";
string yoloClassesFile = yoloBasePath + "coco.names";
string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
string yoloModelWeights = yoloBasePath + "yolov3.weights"; 

vector<string> classes;
ifstream ifs(yoloClassesFile.c_str());
string line;
while (getline(ifs, line)) classes.push_back(line);

// load neural network
cv::dnn::Net net = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

// generate 4D blob from input image
cv::Mat blob;
double scalefactor = 1/255.0;
cv::Size size = cv::Size(416, 416);
cv::Scalar mean = cv::Scalar(0,0,0);
bool swapRB = false;
bool crop = false;
cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

// Scan through all bounding boxes and keep only the ones with high confidence
float confThreshold = 0.20;
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

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

// perform non-maxima suppression
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
vector<int> indices;
cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
