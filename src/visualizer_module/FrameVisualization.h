#ifndef TRACKING_3D_FRAMEVISUALIZATION_H
#define TRACKING_3D_FRAMEVISUALIZATION_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "../datatype/data_structure.cpp"

/**
 * Visualize components on a given frame including PointClouds and detected objects.
 */
class FrameVisualization {
private:
    static const double MAX_POINTCLOUD_DISTANCE; /// Max distnace in Meters to represent a color value for a PointCloud.

    /// Prevent creating an object
    FrameVisualization()=default;
    ~FrameVisualization()=default;

public:
    /**
     * Draw bounding boxes of objects on a frame.
     * @param frame current frame to draw objects.
     * @param frameWithBoundingBox output frame with bounding boxes.
     * @param objects objects to draw on th frame.
     * @param color color of bounding boxes.
     * @param labels vector of all expected labels form the detector.
     * @param thickness bounding box's thickness.
     */
    static void addBoundingBoxes(const cv::Mat& frame, cv::Mat& frameWithBoundingBox, const std::vector<Object*>& objects,
    const cv::Scalar& color, const std::vector<std::string>& labels ,int thickness = 2);

    /**
     * Draw PointClouds of objects in a frame.
     * @param frame frame to draw PointClouds.
     * @param frameWithPointCloud output frame with PointClouds per objects.
     * @param objects objects to draw their PointClouds.
     * @param distanceVariation True if PointCloud distance will be considered, gradient color will be added. Otherwise, fixed blue color.
     */
    static void addPointClouds(const cv::Mat& frame, cv::Mat& frameWithPointCloud, const std::vector<Object*>& objects,
                               const bool distanceVariation);

    /**
     * Visualize frames per milliseconds.
     * @param frame frame to show up.
     * @param delay time in milliseconds between frames.
     * @param title image title.
     */
    static void run(const cv::Mat& frame, const int& delay, const std::string& title);
};


#endif //TRACKING_3D_FRAMEVISUALIZATION_H
