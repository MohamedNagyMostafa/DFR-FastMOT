#ifndef TRACKING_3D_POINTCLOUDVISUALIZATION_H
#define TRACKING_3D_POINTCLOUDVISUALIZATION_H

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <iostream>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "../datatype/data_structure.cpp"

/**
 * @PointCloudVisualization renders PointClouds into 3D space.
 */
class PointCloudVisualization {
public:

    /**
     * @RGB assign colors for viewer's components.
     */
    struct RGB{
    public:
        float red = 0;
        float green = 0;
        float blue  = 0;
        RGB(float red, float green, float blue)
        {
            this->red   = red;
            this->green = green;
            this->blue  = blue;
        }

    };
    /**
     * @Angle @enum contains camera position setups in the viewer.
     * @XY: Far observation based on @_DISTANCE.
     * @TopDown: Top view position.
     */
    enum Angle{
        XY, TopDown
    };

    /**
     * @PointCloudVisualization constructor. Initialize viewer settings, including camera position.
     * @param windowName Pop-up window name.
     * @param angle Camera position with certain angles. Values should be selected from @Angle.
     */
    PointCloudVisualization(const std::string& windowName, Angle angle = XY);

    /**
     * @PointCloudVisualization deconstructor. Clear and close the viewer.
     */
    ~PointCloudVisualization();

    /**
     * Simulate stream of LiDAR PointClouds, rendering on the viewer.
     * @param pointClouds PointClouds to render.
     * @param name Unique name for rendering.
     * @param color PointClouds color. Values should be between 0~1.
     */
    void simulatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, const std::string& name, RGB color = RGB(1, 0, 0));

    /**
     * Change camera position.
     * @param angle New camera position angle. It should be provided from @Angle.
     */
    void changeCameraPosition(Angle angle);

    /**
     * Simulate 3D bounding box for the given objects based on contained PointClouds
     * @param objects list of objects to visualize.
     * @param color bounding box color
     */
    void simulate_3DBoundingBox(const std::vector<Object*> &objects,int streamId, PointCloudVisualization::RGB color = RGB(0, 0, 1));

    void showViewer();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;

    const int DISTANCE = 20;                           // Observation distance. 16 previous
    const RGB BACKGROUND_COLOR = RGB(0, 0, 0);     // Background color, black.

    /**
     * Removes previous PointCloud, clear the viewer.
     */
    void clear();
};


#endif //TRACKING_3D_POINTCLOUDVISUALIZATION_H
