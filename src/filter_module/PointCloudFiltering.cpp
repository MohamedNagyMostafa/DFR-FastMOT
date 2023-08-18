#include "PointCloudFiltering.h"

/**
 * Launch certain filtering method from @FilterType.
 * @param pointClouds PointClouds to filter.
 * @param filteredPointCloud Place to store filtered PointClouds.
 * @param filterType Filter method type, it should be picked from @FilterType.
 * @param bVisual Trace the filtering process.
 */
 void PointCloudFiltering::run(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud,
                               FilterType filterType, bool bVisual)
{
    switch (filterType) {
        case FilterType::CROP_FILTERING:
            cropFiltering(pointClouds, filteredPointCloud);
            break;
        case FilterType::RANSAC_FILTERING:
            ransacFiltering(pointClouds, filteredPointCloud);
            break;
        case FilterType::VOXEL_FILTERING:
            voxelFiltering(pointClouds, filteredPointCloud);
            break;
        default:
            std::cout << "The provided filter is not exist.";
    }
}

/**
 * Launch PointCloud filtering on given objects in order to pick up
 * PointClouds of these objects.
 * @param objects       list of object to obtain PointCloud for.
 * @param filterType    type of filtering. Should be picked from @FilterType.
 */
void PointCloudFiltering::run(std::vector<Object*>& objects,FilterType filterType)
{
    switch(filterType)
    {
        case FilterType::OBJECT_FILTERING:
            objectFiltering(objects);
            break;
    }
}

/**
 *  Filtering using distance properties, cropping based on pre-initialized dimensions.
 * @param pointClouds PointClouds to filter.
 * @param filteredPointCloud Place to store filtered PointClouds.
 */
void PointCloudFiltering::cropFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointClouds,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud)
{

    pcl::CropBox<pcl::PointXYZ> surroundedRegion;

    surroundedRegion.setMax(MAX_CROP_COORDINATES);
    surroundedRegion.setMin(MIN_CROP_COORDINATES);
    surroundedRegion.setInputCloud(pointClouds);
    surroundedRegion.filter(* filteredPointCloud);

}

/**
 *  Filtering by RANSAC-Plane algorithm.
 * @param pointClouds PointClouds to filter.
 * @param filteredPointCloud Place to store filtered PointClouds.
 */
void PointCloudFiltering::ransacFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud)
{

}

/**
 *  Filtering by voxel grid to decline PointClouds resolution.
 * @param pointClouds PointClouds to filter.
 * @param filteredPointCloud Place to store filtered PointClouds.
 */
void PointCloudFiltering::voxelFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(pointClouds);
    voxelGrid.setLeafSize(TOLERATED_DISTANCE_X,TOLERATED_DISTANCE_Y,TOLERATED_DISTANCE_Z);
    voxelGrid.filter(* filteredPointCloud);
}

/**
 * Filter PointCloud based on 2D bounding box of the object. Enclosed
 * PointClouds are assigned to the object.
 * @param objects list of objects to extract PointClouds.
 */
void PointCloudFiltering::objectFiltering(std::vector<Object*>& objects)
{
    for(auto object: objects)
    {
        if(object->pointClouds->empty()) continue;
        // Search Method
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(object->pointClouds);

        // Extracted Clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtraction;

        euclideanClusterExtraction.setInputCloud(object->pointClouds);
        euclideanClusterExtraction.setClusterTolerance(TOLERANCE_DISTANCE_POINTCLOUD);
        euclideanClusterExtraction.setMinClusterSize(MIN_POINTCLOUD);
        euclideanClusterExtraction.setSearchMethod(tree);
        euclideanClusterExtraction.extract(cluster_indices);

        // No PointClouds are detected
        if(cluster_indices.empty()) continue;

        int selectedClusterIndex = 0;

        // In case of multiple clusters, pick maximum one.
        if(cluster_indices.size() > 1)
        {
            int maxClusterSize  = 0;

            for (int i = 0; i < cluster_indices.size(); i++)
            {
                if(maxClusterSize < cluster_indices.at(i).indices.size())
                {
                    maxClusterSize  = cluster_indices.at(i).indices.size();
                    selectedClusterIndex =  i;
                }
            }
        }

        // Enclosed PointCloud
        pcl::PointCloud<pcl::PointXYZ> enclosedPointCloud;

        for(int index : cluster_indices.at(selectedClusterIndex).indices)
            enclosedPointCloud.push_back(object->pointClouds->points[index]);

        enclosedPointCloud.width    = enclosedPointCloud.size();
        enclosedPointCloud.height   = 1;
        enclosedPointCloud.is_dense = true;

        // Extract 3D bounding box.
        pcl::PointXYZ minPoint, maxPoint;

        pcl::getMinMax3D(enclosedPointCloud, minPoint, maxPoint);

        object->_3dBoxLiDAR = new Box3D(minPoint, maxPoint);
        object->_3dBoxLiDAR->found = true;
        object->_3dBoxLiDAR->observationNumber += 1;

    }

}


/// ... Distance Property Settings ... //

// Minimum value for coordinates.
const Eigen::Vector4f PointCloudFiltering::MIN_CROP_COORDINATES(0.5, -20.0, -1.2, 1); // 8 Meters forward, 9 Meters right side, 0.5 Meter upward.
// Maxmimum value for coordinates.
const Eigen::Vector4f PointCloudFiltering::MAX_CROP_COORDINATES(180.0, 20.0, 0.5, 1); // 0.5 Meters forward, 5 Meters left side, 1.1 Meter downward.

/// ... Voxel Grid Settings ... //
// Tolerance distance between PointClouds
const float PointCloudFiltering::TOLERATED_DISTANCE_X = 0.2f;
const float PointCloudFiltering::TOLERATED_DISTANCE_Y = 0.2f;
const float PointCloudFiltering::TOLERATED_DISTANCE_Z = 0.2f;


/// ... Object Filtering Settingsw ... //
const float PointCloudFiltering::TOLERANCE_DISTANCE_POINTCLOUD  = 0.5;
const int PointCloudFiltering::MIN_POINTCLOUD                   = 5;

const float PointCloudFiltering::NO_VALUE = NAN;

