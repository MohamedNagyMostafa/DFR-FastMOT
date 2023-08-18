#include "PointCloudVisualization.h"
#include <fstream>

/**
 * @PointCloudVisualization constructor. Initialize viewer settings, including camera position.
 * @param windowName Pop-up window name.
 * @param angle Camera position with certain angles. Values should be selected from @Angle.
 */
PointCloudVisualization::PointCloudVisualization(const std::string& windowName, PointCloudVisualization::Angle angle)
{
    // Generate 3D viewer.
    pcl::visualization::PCLVisualizer::Ptr viewer_copy(new pcl::visualization::PCLVisualizer(windowName));
    viewer = viewer_copy;

    // init. settings
    viewer->setBackgroundColor(BACKGROUND_COLOR.red, BACKGROUND_COLOR.green, BACKGROUND_COLOR.blue);
    viewer->initCameraParameters();

    // camera angle.
    changeCameraPosition(angle);

    viewer->addCoordinateSystem (1);
    viewer->setShowFPS(true);
}

/**
 * @PointCloudVisualization deconstructor. Clear and close the viewer.
 */
PointCloudVisualization::~PointCloudVisualization()
{
    viewer->close();
}

/**
 * Simulate stream of LiDAR PointClouds, rendering on the viewer.
 * @param pointClouds PointClouds to render.
 * @param name Unique name for rendering.
 * @param color PointClouds color. Values should be between 0~1.
 */
void PointCloudVisualization::simulatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds,
                                                 const std::string& name, PointCloudVisualization::RGB color)
{
    // Clear viewer window.
    clear();

    // Add pointcloud to viewer, and render
    viewer->addPointCloud<pcl::PointXYZ>(pointClouds, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.red, color.green, color.blue, name);
    
}

/**
* Removes previous PointCloud, clear the viewer.
*/
void PointCloudVisualization::clear()
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
}

/**
 * Change camera position.
 * @param angle New camera position angle. It should be provided from @Angle.
 */
void PointCloudVisualization::changeCameraPosition(Angle angle)
{
    switch(angle)
    {
        //case XY: viewer->setCameraPosition(-DISTANCE, -DISTANCE, -DISTANCE, 1, 1, 0); break;
        case XY: viewer->setCameraPosition(-20, 0, 15, 1, 0, 0); break;
        case TopDown: viewer->setCameraPosition(0, 0, DISTANCE, 1, 0, 1); break;
    }
}

void PointCloudVisualization::simulate_3DBoundingBox(const std::vector<Object*> &objects, int streamId,
                                                     PointCloudVisualization::RGB color)
{

    for(const auto object : objects)
    {
        if(object->_3dBoxLiDAR && object->_3dBoxLiDAR->found)
        {
            long unique = random();
            std::string cubeId = "Box " + std::to_string(unique);
            std::string cubeIdEst = "esti " + std::to_string(unique);

//            viewer->addCube(object->_3dBoxLiDAR->x_min, object->_3dBoxLiDAR->x_max,
//                            object->_3dBoxLiDAR->y_min, object->_3dBoxLiDAR->y_max,
//                            0, object->_3dBoxLiDAR->z_max,
//                            color.red, color.green, color.blue, cubeId);
//
            viewer->addCube(object->_3dBoxLiDAR->stateEstimation3D_min(0)-0.1, object->_3dBoxLiDAR->stateEstimation3D_max(0)-0.1,
                            object->_3dBoxLiDAR->stateEstimation3D_min(1)-0.8, object->_3dBoxLiDAR->stateEstimation3D_max(1)-0.8,
                            object->_3dBoxLiDAR->stateEstimation3D_min(2)-1.2, object->_3dBoxLiDAR->stateEstimation3D_max(2)-1.2,
                            0, 255, 0, cubeIdEst);
            viewer->addCube(object->_3dBoxLiDAR->x_min-0.1, object->_3dBoxLiDAR->x_max-0.1,
                            object->_3dBoxLiDAR->y_min-0.8, object->_3dBoxLiDAR->y_max-0.8,
                            object->_3dBoxLiDAR->z_min-1.2, object->_3dBoxLiDAR->z_max-1.2,
                            color.red, color.green, color.blue, cubeId);

            std::cout<<"Object "<< object->trackId << "loc: ("<< object->_3dBoxLiDAR->x_min <<","<< object->_3dBoxLiDAR->y_min <<","<< object->_3dBoxLiDAR->z_min
            << "), ("<<object->_3dBoxLiDAR->x_max <<","<< object->_3dBoxLiDAR->y_max <<","<< object->_3dBoxLiDAR->z_max
                     << ")"<<std::endl;
            std::cout<<"Object "<< object->trackId << "estimation: ("<< object->_3dBoxLiDAR->stateEstimation3D_min(0) <<","<< object->_3dBoxLiDAR->stateEstimation3D_min(1) <<","<< object->_3dBoxLiDAR->stateEstimation3D_min(2)
                     << "), ("<<object->_3dBoxLiDAR->stateEstimation3D_max(0) <<","<< object->_3dBoxLiDAR->stateEstimation3D_max(1) <<","<< object->_3dBoxLiDAR->stateEstimation3D_max(2)<<")"<<std::endl;

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                cubeId);

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                color.red, color.green, color.blue, cubeId);

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cubeId);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cubeIdEst);

            std::string cubeFillId = "boxFill" + std::to_string(unique);

//            viewer->addCube(object->_3dBoxLiDAR->x_min, object->_3dBoxLiDAR->x_max,
//                            object->_3dBoxLiDAR->y_min, object->_3dBoxLiDAR->y_max,
//                            object->_3dBoxLiDAR->z_min, object->_3dBoxLiDAR->z_max,
//                            color.red, color.green, color.blue, cubeFillId);
            viewer->addCube(object->_3dBoxLiDAR->x_min-0.1, object->_3dBoxLiDAR->x_max-0.1,
                            object->_3dBoxLiDAR->y_min-0.8, object->_3dBoxLiDAR->y_max-0.8,
                            object->_3dBoxLiDAR->z_min-1.2, object->_3dBoxLiDAR->z_max-1.2,
                            color.red, color.green, color.blue, cubeFillId);
            viewer->addText3D(std::to_string(object->trackId),
                    pcl::PointXYZ(object->_3dBoxLiDAR->x_max-0.1,object->_3dBoxLiDAR->y_max-0.8,object->_3dBoxLiDAR->z_max-1.2));

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                                cubeFillId);

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                color.red, color.green, color.blue, cubeFillId);

            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, cubeFillId);

        }
    }
}

void PointCloudVisualization::showViewer() {
    // Display pointclouds.
    viewer->spinOnce();
}