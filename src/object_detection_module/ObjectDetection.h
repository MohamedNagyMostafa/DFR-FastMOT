//
// Created by nagy on ١١‏/٣‏/٢٠٢٢.
//

#ifndef TRACKING_3D_OBJECTDETECTION_H
#define TRACKING_3D_OBJECTDETECTION_H

#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include <istream>

#include "../datatype/data_structure.cpp"

/**
 * @class @ObjectDetection uses pretrained detection detectors to locate objects in 2D frame
 * , and assign a unique id for each alongside a bounding box surround the object.
 */
class ObjectDetection {
public:

    /**
     * @struct @PreIdntifiedData holds frame id with all bounding boxes read
     * from a certain file.
     */
    struct PreIdentifiedData
    {
        int frameId;
        std::vector<std::vector<float>> boundingBoxes;

        PreIdentifiedData(int frameId)
        {
            this->frameId = frameId;
        }
    };

    /**
     * @struct @Device available devices (GPU/CPU) to run the model.
     */
    struct Device
    {
        static const int GPU = 0x1;
        static const int CPU = 0x2;
    };

    /**
     * @enum @DetectorType available pretrained detectors to run.
     * @YOLO_DETECTION: Pretrained object detection model (YOLO).
     * @Seg_DETECTION:  Pretrained object detection based on instance segmentation.
     */
    enum DetectorType{
        YOLO_DETECTION,
        SEG_DETECTION,
        PRE_IDENTIFIED
    };

    /**
     * @enum @Class classes considered in object detection module.
     */
    enum Class
    {
        PERSON,
        BYCYCLE,
        CAR,
        MOTORBIKE,
        BUS =5,
        TRUCK = 7
    };

    /**
     * @ObjectDetection YOLO-constructor to initialize YOLO parameters, and load
     * necessary files.
     * @param detectorType Detector type to use. Should be picked up from @DetectorType.
     * @param confidenceThr Confidence threshold for detected objects.
     * @param overlapThr Overlap threshold for non-maximum sup.
     * @param device Device to run the model. Should be chosen from @Device.
     * @param bVisual To monitor detection process.
     */
    ObjectDetection(DetectorType detectorType, float confidenceThr, float overlapThr, int device, std::vector<Class> chosenClasses,bool bVisual = false);

    /**
     * Launch object detector on selected detector given an image.
     * @param image Frame to detect objects.
     * @param objects Vector of type @Object to store detected objects.
     */
    void run(const cv::Mat& image, std::vector<Object*>& objects);

    /**
     * Read object detection data from a file given a frame id.
     * @param frame Represents the intended frame id.
     * @param objects Vector of type @Object to store detection data.
     */
    void run(int frame, std::vector<Object*>& objects);

    /**
     * @return available labels for selected detector model.
     */
    std::vector<std::string> getClasses();

    ~ObjectDetection()=default;

private:
    static const std::string YOLO_BASE_PATH;            /// Base path for YOLO model.
    static const std::string YOLO_WEIGHTS_PATH;         /// YOLO-Weights path.
    static const std::string YOLO_CLASSES_PATH;         /// YOLO-Classes path.
    static const std::string YOLO_CONFIGURATION_PATH;   /// YOLO-Configuration path.
    static const std::string PRE_DETECTED_OBJECTS_PATH; /// Path of pre-detected objects.

    int         usedDetectorType;       /// Selected detector.
    float       confidenceThreshold;    /// Confidence score threshold.
    float       overlapThreshold;       /// Overlap score threshold.
    int         usedDevice;             /// Selected device to operate.
    bool        visual;                 /// Monitor detection operations.

    std::vector<std::string> classes;           /// Object detector labels.
    std::vector<std::string> layers;            /// YOLO-output layers.
    std::vector<Class> chosenClasses;           /// Chosen classes to detect.
    std::vector<PreIdentifiedData> savedData;   /// Stored object detection data from a file.

    cv::dnn::Net yoloNetwork;

    /// Prevent default constructor.
    ObjectDetection()=default;

    /**
     * Initialize YOLO model parameters & model configurations.
     */
    void initializeDetector_YOLO();


    /**
     * Read detection data from a specified file, and store it in a container.
     */
    void initializePreIdentified();

    /**
     * Launch YOLO detector for a given frame.
     * @param image frame to perform object detection.
     * @param objects vector to store detected objects.
     */
    void runYOLO(const cv::Mat& image, std::vector<Object*>& objects);

    /**
     * Check whether a certain class if selected to be detected by the detector or not.
     * @param classId class if to check.
     * @return true if the class is chosen to be detected. Otherwise, returns false.
     */
    bool isSelected(int classId);
};


#endif //TRACKING_3D_OBJECTDETECTION_H
