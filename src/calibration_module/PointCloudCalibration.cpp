//
// Created by nagy on ١٢‏/٣‏/٢٠٢٢.
//

#include "PointCloudCalibration.h"

/**
 * @PointCloudCalibration constructor to load calibration parameters from files, and prepare
 * calibration matr
 * ices.
 * @param calibrationPath calibration files path.
 */

PointCloudCalibration::PointCloudCalibration(const std::string &calibrationPath) {

    P_rect_00   = cv::Mat(3, 4, cv::DataType<double>::type);
    R_rect_00   = cv::Mat(4, 4, cv::DataType<double>::type);
    RT          = cv::Mat(4, 4, cv::DataType<double>::type);

    std::fstream calFile(calibrationPath);

    std::string data;

    std::string pRectStr    = "P2:";
    std::string rRectStr    = "R_rect";
    std::string rtStr       = "Tr_velo_cam";

    while(calFile >> data)
    {
        if(data == pRectStr)
        {
            calFile >> data;
            P_rect_00.at<double>(0,0) = stod(data); calFile >> data;
            P_rect_00.at<double>(0,1) = stod(data); calFile >> data;
            P_rect_00.at<double>(0,2) = stod(data); calFile >> data;
            P_rect_00.at<double>(0,3) = stod(data); calFile >> data;
            P_rect_00.at<double>(1,0) = stod(data); calFile >> data;
            P_rect_00.at<double>(1,1) = stod(data); calFile >> data;
            P_rect_00.at<double>(1,2) = stod(data); calFile >> data;
            P_rect_00.at<double>(1,3) = stod(data); calFile >> data;
            P_rect_00.at<double>(2,0) = stod(data); calFile >> data;
            P_rect_00.at<double>(2,1) = stod(data); calFile >> data;
            P_rect_00.at<double>(2,2) = stod(data); calFile >> data;
            P_rect_00.at<double>(2,3) = stod(data);
        }
        else if(data == rRectStr)
        {
            calFile >> data;
            R_rect_00.at<double>(0,0) = stod(data); calFile >> data;
            R_rect_00.at<double>(0,1) = stod(data); calFile >> data;
            R_rect_00.at<double>(0,2) = stod(data); calFile >> data;
            R_rect_00.at<double>(0,3) = 0.0;
            R_rect_00.at<double>(1,0) = stod(data); calFile >> data;
            R_rect_00.at<double>(1,1) = stod(data); calFile >> data;
            R_rect_00.at<double>(1,2) = stod(data); calFile >> data;
            R_rect_00.at<double>(1,3) = 0.0;
            R_rect_00.at<double>(2,0) = stod(data); calFile >> data;
            R_rect_00.at<double>(2,1) = stod(data); calFile >> data;
            R_rect_00.at<double>(2,2) = stod(data);
            R_rect_00.at<double>(2,3) = 0.0;
        }
        else if(data == rtStr)
        {
            calFile >> data;
            RT.at<double>(0,0) = stod(data); calFile >> data;
            RT.at<double>(0,1) = stod(data); calFile >> data;
            RT.at<double>(0,2) = stod(data); calFile >> data;
            RT.at<double>(0,3) = stod(data); calFile >> data;
            RT.at<double>(1,0) = stod(data); calFile >> data;
            RT.at<double>(1,1) = stod(data); calFile >> data;
            RT.at<double>(1,2) = stod(data); calFile >> data;
            RT.at<double>(1,3) = stod(data); calFile >> data;
            RT.at<double>(2,0) = stod(data); calFile >> data;
            RT.at<double>(2,1) = stod(data); calFile >> data;
            RT.at<double>(2,2) = stod(data); calFile >> data;
            RT.at<double>(2,3) = stod(data);
            RT.at<double>(3,0) = 0.0;
            RT.at<double>(3,1) = 0.0;
            RT.at<double>(3,2) = 0.0;
            RT.at<double>(3,3) = 1.0;

        }
    }

    calFile.close();
}

/**
 * Launch calibration between LiDAR and Camera given 3D PointClouds, and assign individual PointClouds for
 * detected objects based of certain shrink factor.
 * @param pointClouds PointCloud to be projected into 2D dimension.
 * @param objects Detected objects to assign PointClouds to them.
 * @param shrinkFactor Shrink factor to avoid other objects located in the detected bounding box.
 */
void PointCloudCalibration::run(pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, std::vector<Object*>& objects, float shrinkFactor) {

    // 4x1 3D Lidar PointCloud vector
    cv::Mat X(4, 1, cv::DataType<double>::type);
    // 3x1 Projected PointCloud to 2D vector
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for(auto it = pointClouds->begin(); it != pointClouds->end(); it++)
    {

        X.at<double>(0, 0) = (*it).x;
        X.at<double>(1, 0) = (*it).y;
        X.at<double>(2, 0) = (*it).z;
        X.at<double>(3, 0) = 1;

        // projection...
        Y = P_rect_00 * R_rect_00 * RT * X;
        cv::Point pt;

        // pixel point
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        int objectIndex = -1;

        for(int i = 0; i < objects.size(); i++)
        {
            Object* object = objects.at(i);

            cv::Rect smallerBox;
            cv::Rect bBox = object->_2dBox->boundingBox;

            smallerBox.x        = bBox.x + shrinkFactor * bBox.width / 2.0;
            smallerBox.y        = bBox.y + shrinkFactor * bBox.height / 2.0;
            smallerBox.width    = bBox.width * (1 - (shrinkFactor));
            smallerBox.height   = bBox.height * (1 - (shrinkFactor));

            //TODO: No need to remove pointclouds  belong to multiple objects.
            if(smallerBox.contains(pt))
            {
                if (objectIndex == -1)
                {
                    objectIndex = i;
                }
                else
                {
                    objectIndex = -1;
                    break;
                }
            }
        }

        // Prevent multiple assignment.
        if(objectIndex != -1) {

            // 3D PointCloud
            objects.at(objectIndex)->pointClouds->push_back(* it);

            // 2D PointCloud
            objects.at(objectIndex)->pointCloudsInPixels.push_back(pt);
        }
    }
}



