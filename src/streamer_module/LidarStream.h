#ifndef TRACKING_3D_LIDARSTREAM_H
#define TRACKING_3D_LIDARSTREAM_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

#include "../datatype/data_structure.cpp"
#include "../util.cpp"

/**
 * @class LidarStream to handle PointCloud stream coming from LiDAR sensor. Allocating sufficient
 * memory for LiDAR data, and store it into @DataFrame.
 */
class LidarStream {
private:
    std::string mainPath;       // Main directory of the PointCloud dataset.
    std::string filePrefix;     // PointCloud file extension.
    DataFrame* dataFrame;       /// @DataFrame that holds PointClouds for a certain stream.

    int currentStreamId;        // Current stream index.
    bool visual;                // Monitoring interior procedures. [Validation/Testin]

    const int _REPEATED_WIDTH   = 6;                                    // Number of digits in each input file, in KITTI.
    const char _FIXED_INDEX     = '0';                                  // File identifier index.
    const unsigned long _MEMORY_ALLOCATED_SLOTS = 1000000;              // Number of allocated slots in the memory, 4 Megabytes.
    const unsigned long _MEMORY_ALLOCATED_SLOTS_SIZE = sizeof(float);   // Size per slot, 4 bytes.

public:
    /**
     * @struct of acceptable extensions for LiDAR PointCloud files. Accepted extension is .BIN files.
     */
    struct FilePrefix{
        const static std::string BIN_EXTENSION;

        /**
         * @ThrowLiDARPointCloudExtensionException exception for the input extension to make sure
         * the input stream is as expected.
         */
        struct ThrowLiDARPointCloudExtensionException: std::exception{
            const char* what() const throw(){
                return "Unsupported Extension";
            }
        };

        /**
         * To verify the input file extension
         * @param prefix file extension.
         */
        static void acceptable(std::string prefix)
        {
            if(prefix != BIN_EXTENSION)
                throw ThrowLiDARPointCloudExtensionException();
        }
    };

    /**
     * @LidarStream constructor, initialize attributes.
     * @param path Directory of LiDAR PointCloud streams.
     * @param dataFrame Data holder for PointClouds for a certain stream.
     * @param prefix PointClouds file extension. It should be provided by @FilePrefix.
     * @param bVisual Monitoring the internal procedures of PointCloud reading,memory allocation, and storing.
     */
    LidarStream(std::string path, DataFrame* dataFrame, std::string prefix = FilePrefix::BIN_EXTENSION, bool bVisual = false);
    LidarStream(); // Read from LiDAR sensor

    ~LidarStream();

    /**
     * Read PointCloud stream from the assigned directory, allocate 4 Mb for an individual stream. PointClouds in
     * the stream are stored in @LidarPoint from @DataFrame.
     * @return True: if the given directory contains a stream of PointClouds, otherwise returns false.
     *
     */
    bool hasNext();

};



#endif //TRACKING_3D_LIDARSTREAM_H
