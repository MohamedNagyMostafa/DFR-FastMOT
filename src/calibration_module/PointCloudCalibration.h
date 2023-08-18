#ifndef TRACKING_3D_POINTCLOUDCALIBRATION_H
#define TRACKING_3D_POINTCLOUDCALIBRATION_H

#include <iostream>
#include <fstream>

#include "../datatype/data_structure.cpp"

/**
 * @class @PointCloudCalibration responsible for calibration between Camera and LiDAR sensors,
 *  and assign PointClouds for 2D detected objects.
 */
class PointCloudCalibration {

private:
    /// Calibration Matrices ///
    cv::Mat P_rect_00;
    cv::Mat R_rect_00;
    cv::Mat RT;

public:
    /**
     * @PointCloudCalibration constructor to load calibration parameters from files, and prepare
     * calibration matrices.
     * @param calibrationPath calibration files path.
     */
    explicit PointCloudCalibration(const std::string& calibrationPath);
    ~PointCloudCalibration()= default;

    /**
     * Launch calibration between LiDAR and Camera given 3D PointClouds, and assign individual PointClouds for
     * detected objects based of certain shrink factor.
     * @param pointClouds PointCloud to be projected into 2D dimension.
     * @param objects Detected objects to assign PointClouds to them.
     * @param shrinkFactor Shrink factor to avoid other objects located in the detected bounding box.
     */
    void run(pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, std::vector<Object*>& objects, float shrinkFactor = 0);
};


#endif //TRACKING_3D_POINTCLOUDCALIBRATION_H
