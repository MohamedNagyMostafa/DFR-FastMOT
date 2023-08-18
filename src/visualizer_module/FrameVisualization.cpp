#include <iostream>
#include "FrameVisualization.h"

/**
 * Draw bounding boxes of objects on a frame.
 * @param frame current frame to draw objects.
 * @param frameWithBoundingBox output frame with bounding boxes.
 * @param objects objects to draw on th frame.
 * @param color color of bounding boxes.
 * @param labels vector of all expected labels form the detector.
 * @param thickness bounding box's thickness.
 */
void FrameVisualization::addBoundingBoxes(const cv::Mat& frame, cv::Mat& frameWithBoundingBox, const std::vector<Object*>& objects,
                                          const cv::Scalar& color, const std::vector<std::string>& labels ,int thickness) {
    frameWithBoundingBox = frame.clone();

    for (const auto &object: objects) {
        if(! object->_2dBox) continue;
        // Draw bounding box.
        cv::Rect bBox = object->_2dBox->boundingBox;

        cv::Point leftTop = cv::Point(bBox.x, bBox.y);
        cv::Point rightBottom = cv::Point(bBox.x + bBox.width, bBox.y + bBox.height);

        cv::rectangle(frameWithBoundingBox, leftTop, rightBottom, color, thickness);
        std::cout<<"visual ... "<< std::endl;
        std::cout<<"(" << leftTop.x << " , "<< leftTop.y
                 <<", (" << rightBottom.x << " , "<< rightBottom.y <<")"
                 <<std::endl;
        // estimation Bb.
        cv::rectangle(frameWithBoundingBox,
                      cv::Point(object->_2dBox->stateEstimation2D_topLeft(0), object->_2dBox->stateEstimation2D_topLeft(1))
                      , cv::Point(object->_2dBox->stateEstimation2D_rightBottom(0), object->_2dBox->stateEstimation2D_rightBottom(1))
                      , cv::Scalar (0, 0, 255)
                      , thickness);

        // Add text for bounding box.
        std::string label = cv::format("%.2f", object->confidence);
        label ="ID:"+ std::to_string(object->trackId);
        if ((std::to_string(object->trackId)).size() > 3) label = "";
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);

        int top = cv::max(bBox.y, labelSize.height);

        cv::Point leftTop_text = cv::Point(bBox.x, top - round(1.5 * labelSize.height));
        cv::Point rightBottom_text = cv::Point(bBox.x + round(1.5 * labelSize.width), top + baseLine);

        // White rectangle surround the text.
        cv::rectangle(frameWithBoundingBox,
                      leftTop_text,
                      rightBottom_text,
                      cv::Scalar(255, 255, 255),
                      cv::FILLED);

        cv::putText(frameWithBoundingBox,
                    label,
                    cv::Point(bBox.x, top),
                    cv::FONT_ITALIC,
                    0.75,
                    cv::Scalar(0, 0, 0),
                    1);

    }

}

/**
 * Visualize frames per milliseconds.
 * @param frame frame to show up.
 * @param delay time in milliseconds between frames.
 * @param title image title.
 */
void FrameVisualization::run(const cv::Mat& frame, const int& delay, const std::string& title)
{
        cv::imshow(title, frame);
        cv::waitKey(delay);
}

/**
 * Draw PointClouds of objects in a frame.
 * @param frame frame to draw PointClouds.
 * @param frameWithPointCloud output frame with PointClouds per objects.
 * @param objects objects to draw their PointClouds.
 * @param distanceVariation True if PointCloud distance will be considered, gradient color will be added. Otherwise, fixed blue color.
 */
void FrameVisualization::addPointClouds(const cv::Mat &frame, cv::Mat &frameWithPointCloud,
                                        const std::vector<Object*> &objects, const bool distanceVariation)
{
    frameWithPointCloud = frame.clone();

    for(const Object* object : objects)
    {
        for(int i = 0; i < object->pointCloudsInPixels.size(); i++)
        {
            int green   = 0;
            int red     = 255;

            if(distanceVariation)
            {
                double distance     = abs(object->pointClouds->at(i).x);

                int red     = (int) 255 * abs(distance - MAX_POINTCLOUD_DISTANCE)/ MAX_POINTCLOUD_DISTANCE;
                int green   = 255 - red;
            }

            cv::circle(frameWithPointCloud, object->pointCloudsInPixels[i], 5, cv::Scalar(0, green, red), -1);
        }
    }

}

/// Max distnace in Meters to represent a color value for a PointCloud.
const double FrameVisualization::MAX_POINTCLOUD_DISTANCE = 20.0;

