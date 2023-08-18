
#ifndef LIST_H_
#define LIST_H_

#include <opencv2/core/mat.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>

/**
 * @LidarPoint datatype represents an individual PointCloud in 3D space.
 * @x: x-dimension in 3D space.
 * @y: y-dimension in 3D space.
 * @z: z-dimension in 3D space.
 * @r: rotation in 3D space.
 */
struct LidarPoint{
public:
    LidarPoint(double x, double y, double z, double r)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->r = r;
    }
    double x, y, z, r;
};


struct Pose
{
    double x;
    double y;
    double z;

    double xv;
    double yv;

    double yr;

    Pose() {}
    Pose(double x, double y, double z, double rotation)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->xv = 0;
        this->yv = 0;
        this->yr = rotation;
    }

    Pose(double x, double y, double z, double xv, double yv)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->xv = xv;
        this->yv = yv;
    }
};
/**
 * @Box3D datatype describes 3D bounding box by maximum and minimum of each coordinates system. (x/y/z).
 */
struct Box3D
{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;

    Eigen::VectorXd stateEstimation3D_min; /// (px, py, vx, vy, ax, ay)
    Eigen::VectorXd stateEstimation3D_max;

    bool found;
    int observationNumber;
    int lastObservation;

    Box3D()
    {
        stateEstimation3D_min = Eigen::VectorXd(9);
        stateEstimation3D_min = stateEstimation3D_min.setOnes() * -1;
        stateEstimation3D_max = Eigen::VectorXd(9);
        stateEstimation3D_max = stateEstimation3D_max.setOnes() * -1;

        found = true;
        observationNumber = 0;
        lastObservation   = 0;
    }
    /**
     * Initialize bounding box coordinates by picking up minimum and maximum point
     * in 3D space.
     * @param minPoint
     * @param maxPoint
     */
    Box3D(pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint)
    {
        x_min = minPoint.x;
        x_max = maxPoint.x;
        y_min = minPoint.y;
        y_max = maxPoint.y;
        z_min = minPoint.z;
        z_max = maxPoint.z;

        stateEstimation3D_min = Eigen::VectorXd(9);
        stateEstimation3D_min = stateEstimation3D_min.setOnes() * -1;
        stateEstimation3D_max = Eigen::VectorXd(9);
        stateEstimation3D_max = stateEstimation3D_max.setOnes() * -1;

        found = true;
        observationNumber = 0;
        lastObservation   = 0;
    }

//    bool isFlipped()
//    {
//        return stateEstimation3D_min(2) > stateEstimation3D_max(1);
//    }
};

/**
 * @Box2D datatype describes 2D bounding box by maximum and minimum of each coordinates system. (x/y).
 */
struct Box2D
{
    cv::Rect boundingBox;

    Eigen::VectorXd stateEstimation2D_topLeft;
    Eigen::VectorXd stateEstimation2D_rightBottom;

    bool found;

    int observationNumber;
    int lastObservation;
    /**
     * Initialize bounding box coordinates by picking up minimum and maximum point
     * in 2D space.
     */
    Box2D()
    {
        found   = true;
        observationNumber = 0;
        lastObservation   = 0;

        stateEstimation2D_topLeft = Eigen::VectorXd(6);
        stateEstimation2D_topLeft = stateEstimation2D_topLeft.setOnes() * -1;
        stateEstimation2D_rightBottom = Eigen::VectorXd(6);
        stateEstimation2D_rightBottom = stateEstimation2D_rightBottom.setOnes() * -1;

    }

//    bool isFlipped()
//    {
//        return stateEstimation2D_topLeft(0) > stateEstimation2D_rightBottom(0) ||
//        stateEstimation2D_topLeft(1) > stateEstimation2D_rightBottom(1);
//    }
};

/**
 * @Object datatype represents objects detected by sensors.
 * @param id            unique id to identify the object.
 * @param classId       id refers to the type class of the object.
 * @param detectionId   unique detection id for the object.
 * @param trackId       unique id to track the object.
 * @param confidence    confidence score for the detected object
 * @param _2dBox        2D-Bounding box of the object in 2D image frame.
 * @param _3dBox        3D-Bounding box of the object in 3D LiDAR PointCloud.
 */
struct Object{
    int id;
    int classId;
    int detectionId;
    int appearance;
    float score;
    float alpha;
    float rt;
    long trackId;
    double confidence;


    Box2D*  _2dBox;
    Box3D*  _3dBoxLiDAR;
    Box3D*  _3dBoxIMU;

    std::vector<cv::KeyPoint> keypoints;
    std::vector<std::pair<int,cv::Mat>> scoredKeypoints;
    cv::Mat* descriptor;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds;
    std::vector<cv::Point> pointCloudsInPixels;

    Object()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_cp(new pcl::PointCloud<pcl::PointXYZ>);
        pointClouds         = pointCloud_cp;
        _3dBoxLiDAR         = nullptr;
        _3dBoxIMU           = nullptr;
        _2dBox              = nullptr;
        descriptor          = new cv::Mat();
        trackId             = random();
        appearance          = 0;
        score               = 0;
        alpha               = -10;
        rt                  = -10;

    }

    int lastObservation()
    {
        int out = 0;
        if(_3dBoxLiDAR)
            out = _3dBoxLiDAR->lastObservation;

        if(_2dBox)
        {
            if(_3dBoxLiDAR)out = std::min(out, _2dBox->lastObservation);
            else out = _2dBox->lastObservation;
        }


        return out;
    }
    cv::Mat  getTopFeatures(int top)
    {
        if(descriptor->rows > top)
            return (*descriptor)(cv::Range(0,top), cv::Range(0, descriptor->cols));

        return (*descriptor);
    }
    ~Object()
    {
        if(!_3dBoxLiDAR)        delete _3dBoxLiDAR;
        if(!_3dBoxIMU)          delete _3dBoxIMU;
        if(!_2dBox)             delete _2dBox;
        if(!descriptor)         delete descriptor;
    }
};

/**
 * Data holder for an individual stream. Corresponding image is stored alongside
 * corresponding PointClouds inside a vector of @LidarPoint.
 */
struct DataFrame{
public:
    cv::Mat* imageFrame;                                    // Image for a given stream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds;        // PointClouds for a given stream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudsFiltered;// Filtered PointClouds.
    std::vector<Object* > objects;                            // Holding detected objects in current frame.

    DataFrame()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_cp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered_cp(new pcl::PointCloud<pcl::PointXYZ>);

        pointClouds = pointCloud_cp;
        pointCloudsFiltered = pointCloudFiltered_cp;

        imageFrame          = new cv::Mat();
    }

    ~DataFrame()
    {
        delete imageFrame;
        for(auto object: objects) delete object;
    }

    /**
     * Clear data for next stream
     */
    void reset()
    {
        pointClouds->clear();
        pointCloudsFiltered->clear();
        std::cout<<"ddd"<<std::endl;
       //for(auto object: objects) if(object->trackId >= 100) delete object;
        std::cout<<"done"<<std::endl;

        objects.clear();
        std::cout<<"way"<<std::endl;

    }
};



#endif

