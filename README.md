# DFR-FastMOT: A Robust Tracker for Fast Multi-Object Tracking with Sensor Fusion

Welcome to the DFR-FastMOT repository, showcasing the complete implementation of our innovative tracking model. The research is presented in [IEEE ICRA23 conference](https://ieeexplore.ieee.org/document/10160328). For those interested, the pre-print version is available on [arXiv](https://arxiv.org/abs/2302.14807).

## Abstract
Multi-object tracking (MOT) in dynamic environments is crucial for safe autonomous navigation. One major challenge in MOT arises when objects are occluded, rendering them temporarily invisible. Existing MOT methods store object information for recovery post-occlusion, striking a balance between memory efficiency and tracking accuracy. However, such methods often struggle with prolonged occlusions. In this paper, we propose DFR-FastMOT—a lightweight tracking method using camera and LiDAR data fusion. Our approach leverages algebraic object association and fusion, optimizing computation time and enabling long-term memory. DFR-FastMOT excels over both learning and non-learning benchmarks, surpassing them by approximately 3% and 4% in MOTA, respectively. We subject our method to extensive occlusion simulations with varying detection distortion levels, showcasing its superiority over current state-of-the-art methods. Remarkably, our framework processes 7,763 frames in just 1.48 seconds—seven times faster than recent benchmarks.

[![Watch the video](https://img.youtube.com/vi/CLDxqTojz6o/0.jpg)](https://youtu.be/CLDxqTojz6o)

## Getting Started
Our implementation is written entirely in C++/11. To run the code on your machine, make sure to download the required libraries and dependencies. The following instructions will guide you through the setup process. For any inquiries, feel free to reach out to us using the contact details provided at the end of this documentation.

### Required Libraries
* Minimum cmake version: 3.16
* OpenCV 4.1
* PCL 1.10
* Eigen3

### Dataset
DFR-FastMOT can work with various dataset formats, as long as they adhere to the [KITTI dataset](https://www.cvlibs.net/datasets/kitti/eval_tracking.php) structure for multi-object tracking. Update the `BASE_DIR` variable in the `main.cpp` file to point to your dataset directory. The dataset directory structure should match the following:

```
/dataset
├── Camera
│   └── training
│       ├── image_02
│       │   ├── 0000
│       │   │   ├── ...
│       │   ├── xxxx
│       │       ├── ...
│       └── label_02
│           ├── 0000
│           │   ├── ...
│           ├── xxxx
│               ├── ...
└── LiDAR
    └── training
        ├── velodyne
        │   ├── 0000
        │   │   ├── ...
        │   ├── xxxx
        │       ├── ...
        └── calib
            ├── 0000.txt
            ├── ...
            └── xxxx.txt
```

Make sure to update the `CURRENT_STREAM` and `BASE_DIR` variables in the `main.cpp` file accordingly.

```c++
const string CURRENT_STREAM = "0002"; // Change it base on your intented stream. In this case, it will run stream 0002 in KITTI dataset.
const string BASE_DIR       = "/home/mohamed/Desktop/Thesis Prot/dataset/"; // Locate the dataset (BASE) directory on your machine.
```

### 2D/3D Detection Inputs
DFR-FastMOT employs the `Detection Module` to conduct `YOLOv3` detection on 2D images. It projects 2D bounding boxes into the 3D point cloud to achieve 3D detection. Additionally, the model accepts external detection inputs in `.txt` format. To provide these inputs, place 2D detection files in the `src/advanced_detection/det_2d/` directory, and 3D detection files in the `src/advanced_detection/det_3d/` directory. The filenames should match the stream names in your dataset. Refer to the examples in the directories for guidance.

Sample format for 2D detections in `.txt` file:

| frame_id | 2d_bbox (x1,y1,x2,y2)            | score   |
|:--------:|---------------------------------|---------|
| 0        | 296.021,160.173,452.297,288.372 | 0.52923 |

Sample format for 3D detections in `.txt` file:

| frame_id | type_id | 2d_bbox (x1,y1,x2,y2)                | score   | 3d_bbox (y_max,z_max,x_max, y_min, z_min, x_min)           | rotation | alpha   |
|:--------:|---------|-------------------------------------|---------|-----------------------------------------------------------|----------|---------|
| 0        | 2       | 298.3125,165.1800,458.2292,293.4391 | 8.2981  | 1.9605,1.8137,4.7549,-4.5720,1.8435,13.5308             | -2.1125  | -1.7867 |

### Modules Overview
#### Association Module:
This module orchestrates the association and fusion of tracked objects based on the methods detailed in the paper.
#### Calibration Module:
Responsible for projecting bounding boxes into 2D/3D sensor coordinates.
#### Filter Module:
Performs point cloud filters like cropping and voxel grid operations on the input point cloud.
#### Memory Module:
Stores object information in internal memory for subsequent utilization.
#### Object Detection Module:
Detects objects using the YOLOv3 model, if not provided as input. It also generates 3D detections through projection and 3D neighborhood clustering.
#### Streamer Module:
Simulates real-time data flow from cameras and LiDAR sensors by sequentially streaming data.
#### Tracking Module:
Utilizes the Kalman filter state estimation algorithm to predict the next state for all objects in memory based on constant acceleration.
#### Visualizer Module:
Enables real-time visualization of 2D/3D tracking outputs.

### Running the Model
Once you've completed the setup, simply execute the `main.cpp` file. This will display two windows—one for 2D camera tracking visualization and another for 3D point cloud tracking. See the example below for reference.

![Visualization Example](https://github.com/wangxiyang2022/DeepFusionMOT/assets/20774864/f70065c6-fa55-45ae-aeaa-924bbb964db2)

### Evaluation
For accurate tracking output evaluation, we recommend using our evaluation repository available [HERE](https://github.com/MohamedNagyMostafa/KITTI-MOT.Bench-Evals). The tracking output adheres to the KITTI format and is stored in the `src/tracking` directory.

| frame | type | truncated | occluded | alpha | 2d_bbox | 3d_dimensions | 3d_location | rotation_y | score |
|:-----:|:----:|:---------:|:--------:|:-----:|:-------:|:------------:|:-----------:|:----------:|:-----:|
|   0   | Car  |    0      |    0     | -1  |(255,182,0,23)|(1.24,2,3,0.9)|(2.3,5.9,1.2)|    -35    |   0   |

### Citation
If you find our implementation valuable for your research, kindly consider citing our paper:

```
@INPROCEEDINGS{10160328,
  author={Nagy, Mohamed and Khonji, Majid and Dias, Jorge and Javed, Sajid},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={DFR-FastMOT: Detection Failure Resistant Tracker for Fast Multi-Object Tracking Based on Sensor Fusion}, 
  year={2023},
  volume={},
  number={},
  pages={827-833},
  doi={10.1109/ICRA48891.2023.10160328}}
```

### Contact Us
For inquiries related to the paper or the implementation, feel free to reach out to us at: mohamed.nagy@ieee.org
