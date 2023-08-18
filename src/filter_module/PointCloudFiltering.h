//
// Created by nagy on ١١‏/٣‏/٢٠٢٢.
//

#ifndef TRACKING_3D_POINTCLOUDFILTERING_H
#define TRACKING_3D_POINTCLOUDFILTERING_H

#include <iostream>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "../datatype/data_structure.cpp"

/**
 * @class @PointCloudFiltering to crop PointClouds, from high resolution to low resolution using
 * one of the following methods:
 * * Crop by RANSAC-plane.
 * * Crop by distance properties.
 * * Crop by voxel grid.
 * Any kind of filtering will be processed throughout this class.
 */
class PointCloudFiltering {
public:
    const static float NO_VALUE; /// Refers to no value is assigned for limited dimensions.

    /**
     * @enum @FilterType list of available filtering methods.
     * @RANSAC_FILTERING:   Filter PoinClouds by 3D-RANSAC plane, excluding surfaces.
     * @CROP_FILLTERING:    Filter PointClouds by assigning fixed distance properties in x, y, z coordinates (in Meter).
     * @VOXEL_FILTERING:    Filter PointClouds using voxel grid by degrade PointCloud resolution.
     */
    enum FilterType{
        RANSAC_FILTERING,
        CROP_FILTERING,
        VOXEL_FILTERING,
        OBJECT_FILTERING,
    };

    /**
     * Launch certain filtering method from @FilterType.
     * @param pointClouds PointClouds to filter.
     * @param filteredPointCloud Place to store filtered PointClouds.
     * @param filterType Filter method type, it should be picked from @FilterType.
     * @param bVisual Trace the filtering process.
     */
    static void run(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud, FilterType filterType, bool bVisual = false);

    /**
      * Launch PointCloud filtering on given objects in order to pick up
      * PointClouds of these objects.
      * @param objects       list of object to obtain PointCloud for.
      * @param filterType    type of filtering. Should be picked from @FilterType.
      */
    static void run(std::vector<Object*>& objects, FilterType filterType);

private:
    /// Prevent initialization.
    PointCloudFiltering()= default;
    ~PointCloudFiltering()= default;

    /// Crop Filtering Settings
    static const Eigen::Vector4f MIN_CROP_COORDINATES;  /// Minimum value for coordinates.
    static const Eigen::Vector4f MAX_CROP_COORDINATES;  /// Maximum value for coordinates.

    /// Voxel Grid Settings
    static const float TOLERATED_DISTANCE_X;
    static const float TOLERATED_DISTANCE_Y;
    static const float TOLERATED_DISTANCE_Z;

    /// RANSAC Settings

    /// Object Filtering Settings
    static const float  TOLERANCE_DISTANCE_POINTCLOUD;
    static const int    MIN_POINTCLOUD;


    /**
     *  Filtering by RANSAC-Plane algorithm.
     * @param pointClouds PointClouds to filter.
     * @param filteredPointCloud Place to store filtered PointClouds.
     */
    static void ransacFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud);

    /**
     *  Filtering using distance properties, cropping based on pre-initialized dimensions.
     * @param pointClouds PointClouds to filter.
     * @param filteredPointCloud Place to store filtered PointClouds.
     */
    static void cropFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointClouds, pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud);

    /**
     *  Filtering by voxel grid to decline PointClouds resolution.
     * @param pointClouds PointClouds to filter.
     * @param filteredPointCloud Place to store filtered PointClouds.
     */
    static void voxelFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud);

    /**
     * Filter PointCloud based on 2D bounding box of the object. Enclosed
     * PointClouds are assigned to the object.
     * @param objects list of objects to extract PointClouds.
     */
    static void objectFiltering(std::vector<Object*>& objects);

};


#endif //TRACKING_3D_POINTCLOUDFILTERING_H
