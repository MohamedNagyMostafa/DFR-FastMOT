#include "ObjectDetection.h"

/**
 * @ObjectDetection YOLO-constructor to initialize YOLO parameters, and load
 * necessary files.
 * @param detectorType Detector type to use. Should be picked up from @DetectorType.
 * @param confidenceThr Confidence threshold for detected objects.
 * @param overlapThr Overlap threshold for non-maximum sup.
 * @param device Device to run the model. Should be chosen from @Device.
 * @param bVisual To monitor detection process.
 */
ObjectDetection::ObjectDetection(DetectorType detectorType, float confidenceThr, float overlapThr, int device, std::vector<ObjectDetection::Class> chosenClasses, bool bVisual)
{
    confidenceThreshold = confidenceThr;
    overlapThreshold    = overlapThr;
    usedDevice          = device;
    visual              = bVisual;
    usedDetectorType    = detectorType;
    this->chosenClasses = chosenClasses;

    switch (usedDetectorType)
    {

        case YOLO_DETECTION:
            initializeDetector_YOLO();
            break;

        case PRE_IDENTIFIED:
            initializePreIdentified();

        default:
            std::cout<< "The chosen detector is not defined."<<std::endl;
    }
}


void ObjectDetection::initializePreIdentified()
{

    std::ifstream inFile(PRE_DETECTED_OBJECTS_PATH);

    int frameId;

    // Go over ea
    while(inFile>>frameId)
    {
        std::vector<float>boundingBox;
        float value;

        // Read bounding box values
        inFile>>value;
        boundingBox.push_back(value);
        inFile>>value;
        boundingBox.push_back(value);
        inFile>>value;
        boundingBox.push_back(value);
        inFile>>value;
        boundingBox.push_back(value);
        inFile>>value;

        PreIdentifiedData preIdentifiedData(frameId);

        // Store the bounding box.
        if(savedData.empty())
        {
            preIdentifiedData.boundingBoxes.push_back(boundingBox);
            savedData.push_back(preIdentifiedData);
        }
        else
            if(savedData.at(savedData.size() -1).frameId == frameId)
                savedData.at(savedData.size() -1).boundingBoxes.push_back(boundingBox);
            else
            {
                preIdentifiedData.boundingBoxes.push_back(boundingBox);
                savedData.push_back(preIdentifiedData);
            }
    }

}


/**
 * Initialize YOLO model parameters & model configurations.
 */
void ObjectDetection::initializeDetector_YOLO()
{
    // Read classes
    std::ifstream classFile((YOLO_BASE_PATH + YOLO_CLASSES_PATH).c_str());
    std::string class_;

    while(getline(classFile, class_))
    {
        classes.push_back(class_);
    }

    // Load yolo network
    yoloNetwork = cv::dnn::readNetFromDarknet(YOLO_BASE_PATH + YOLO_CONFIGURATION_PATH,
                                              YOLO_BASE_PATH + YOLO_WEIGHTS_PATH);
    yoloNetwork.setPreferableBackend((usedDevice == Device::GPU)? cv::dnn::DNN_BACKEND_CUDA : cv::dnn::DNN_BACKEND_OPENCV);
    yoloNetwork.setPreferableTarget((usedDevice == Device::GPU)? cv::dnn::DNN_TARGET_CUDA : cv::dnn::DNN_TARGET_CPU);

    // Extract output layers
    std::vector<int> outputLayers = yoloNetwork.getUnconnectedOutLayers();
    std::vector<std::string> layersNames = yoloNetwork.getLayerNames();

    for(int outputLayer : outputLayers)
    {
        layers.push_back(layersNames[outputLayer - 1]);
    }
}

/**
 * Launch object detector on selected detector given an image.
 * @param image Frame to detect objects.
 * @param objects Vector of type @Object to store detected objects.
 */
void ObjectDetection::run(const cv::Mat& image, std::vector<Object*> &objects)
{
    switch (usedDetectorType)
    {
        case YOLO_DETECTION:
            runYOLO(image, objects);
            break;
    }
}

/**
 * Read object detection data from a file given a frame id.
 * @param frame Represents the intended frame id.
 * @param objects Vector of type @Object to store detection data.
 */
void ObjectDetection::run(int frame, std::vector<Object*>& objects)
{
    int currIndex = -1;

    // Check whether the pointed frame is existed or not. If exist pick first index.
    for(PreIdentifiedData preIdentifiedData: savedData)
    {
        currIndex++;

        if(frame != preIdentifiedData.frameId && frame > preIdentifiedData.frameId)
            continue;
        else if(frame == preIdentifiedData.frameId)
            break;
        else
            return;

    }
    std::cout<<"readed " <<std::endl;

    // Index not exist, break.
    if(currIndex == -1) return;
    std::cout<<"readed " <<std::endl;

    // Store objects.
    for(int i =0 ; i< savedData.at(currIndex).boundingBoxes.size(); i++)
    {
        auto info = savedData.at(currIndex).boundingBoxes.at(i);

        auto obj = new Object();
        obj->_2dBox = new Box2D();

        obj->_2dBox->boundingBox.x        = info.at(0);
        obj->_2dBox->boundingBox.y        = info.at(1);
        obj->_2dBox->boundingBox.width    = abs(info.at(0)- info.at(2));
        obj->_2dBox->boundingBox.height   = abs(info.at(1)- info.at(3));
        obj->confidence                   = 0;
        obj->classId                      = ObjectDetection::CAR;
        obj->detectionId                  = i;


        objects.push_back(obj);
    }
}

/**
 * Launch YOLO detector for a given frame.
 * @param image frame to perform object detection.
 * @param objects vector to store detected objects.
 */
void ObjectDetection::runYOLO(const cv::Mat &image, std::vector<Object*> &objects)
{
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;

    // Image to blob.
    cv::Mat inputBlob;
    std::vector<cv::Mat> outputBlob;
    cv::dnn::blobFromImage(image, inputBlob, 1/255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0));

    yoloNetwork.setInput(inputBlob);
    yoloNetwork.forward(outputBlob, layers);

    // Add bounding boxes.
    for(auto & item : outputBlob)
    {
        auto* data = (float*) item.data;
        for(size_t j = 0; j < item.rows; j++, data += item.cols)
        {
            // Extract scores for the object
            cv::Mat scores = item.row(j).colRange(5 ,item.cols);
            cv::Point classId;
            double confidence;

            cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
            // Skip classes that not selected
            if(!isSelected(classId.x)) continue;

            if(confidence > confidenceThreshold)
            {
                int cx, cy;
                cv::Rect box;

                cx = (int)(data[0] * image.cols);
                cy = (int)(data[1] * image.rows);

                // Add object information.
                box.width   = (int)(data[2] * image.cols);
                box.height  = (int)(data[3] * image.rows);
                box.x       = abs(cx - box.width / 2.0) ;
                box.y       = abs(cy - box.height / 2.0 );

                boxes.push_back(box);
                confidences.push_back((float) confidence);
                classIds.push_back(classId.x);
            }
        }
    }

    // Perform non-Maxima supression.
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences,  confidenceThreshold, overlapThreshold, indices);

    for(int & indice : indices)
    {
        auto obj = new Object();
        obj->_2dBox = new Box2D();

        obj->_2dBox->boundingBox.x        = boxes[indice].x;
        obj->_2dBox->boundingBox.y        = boxes[indice].y;
        obj->_2dBox->boundingBox.width    = (boxes[indice].width  + boxes[indice].x <= image.cols)?boxes[indice].width : image.cols - boxes[indice].x;
        obj->_2dBox->boundingBox.height   = (boxes[indice].height  + boxes[indice].y <= image.rows)?boxes[indice].height : image.rows - boxes[indice].y;
        obj->confidence                   = confidences[indice];
        obj->classId                      = classIds[indice];
        obj->detectionId                  = indice;
        obj->_2dBox->observationNumber    = 1;

        objects.push_back(obj);
    }
}

/**
 * @return available labels for selected detector model.
 */
std::vector<std::string> ObjectDetection::getClasses()
{
    return classes;
}

/**
 * Check whether a certain class if selected to be detected by the detector or not.
 * @param classId class if to check.
 * @return true if the class is chosen to be detected. Otherwise, returns false.
*/
bool ObjectDetection::isSelected(int classId) {
    for(auto it = chosenClasses.begin(); it != chosenClasses.end(); it++)
    {
        if(classId == *it) return true;
    }
    return false;
}

/// ...... Path ..... ///
//TODO: modify YOLO_BASE_PATH base on your machine.
const std::string ObjectDetection::YOLO_BASE_PATH           = "/home/mohamed/Desktop/Thesis Project/Tracking-3D/object_detection_module/models/yolo/";
const std::string ObjectDetection::YOLO_WEIGHTS_PATH        = "yolov3.weights";
const std::string ObjectDetection::YOLO_CLASSES_PATH        = "coco.names";
const std::string ObjectDetection::YOLO_CONFIGURATION_PATH  = "yolov3.cfg";
const std::string ObjectDetection::PRE_DETECTED_OBJECTS_PATH= "../detection_data/0006.txt";


