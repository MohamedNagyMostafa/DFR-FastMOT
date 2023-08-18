cmake_minimum_required(VERSION 3.16)
project(Tracking_3D)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV 4.1 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(TestTracker3D test/KF_test.cpp src/datatype/data_structure.cpp src/tracking_module/KF_Tracking.cpp src/tracking_module/KF_Tracking.h)
add_executable(Tracking_3D src/main.cpp src/streamer_module/ImageStream.cpp src/streamer_module/ImageStream.h src/datatype/data_structure.cpp src/streamer_module/LidarStream.cpp src/streamer_module/LidarStream.h src/util.cpp src/visualizer_module/PointCloudVisualization.cpp src/visualizer_module/PointCloudVisualization.h src/filter_module/PointCloudFiltering.cpp src/filter_module/PointCloudFiltering.h src/object_detection_module/ObjectDetection.cpp src/object_detection_module/ObjectDetection.h src/visualizer_module/FrameVisualization.cpp src/visualizer_module/FrameVisualization.h src/calibration_module/PointCloudCalibration.cpp src/calibration_module/PointCloudCalibration.h src/memory_module/MemoryManagement.cpp src/memory_module/MemoryManagement.h src/association_module/ObjectAssociation.cpp src/association_module/ObjectAssociation.h  src/tracking_module/KF_Tracking.cpp src/tracking_module/KF_Tracking.h)

target_link_libraries (Tracking_3D ${OpenCV_LIBS})
target_link_libraries (Tracking_3D ${PCL_LIBRARIES})
target_link_libraries (TestTracker3D ${OpenCV_LIBS})
target_link_libraries (TestTracker3D ${PCL_LIBRARIES})
