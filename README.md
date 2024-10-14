<div align="center">
    <h1>Kinematic-ICP</h1>
    <a href="https://github.com/PRBonn/kinematic-icp/releases"><img src="https://img.shields.io/github/v/release/PRBonn/kinematic-icp?label=version" /></a>
    <a href="https://github.com/PRBonn/kinematic-icp/blob/main/LICENSE"><img src="https://img.shields.io/github/license/PRBonn/kinematic-icp" /></a>
    <a href="https://github.com/PRBonn/kinematic-icp/blob/main/"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <br />
    <br />
    <a href=https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/kissteam2025icra.pdf>Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/PRBonn/kinematic-icp/issues>Contact Us</a>
  <br />
  <br />

[Kinematic-ICP](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/kissteam2025icra.pdf) is a LiDAR odometry approach that explicitly incorporates the kinematic constraints of mobile robots into the classic point-to-point ICP algorithm.

<img src="https://github.com/user-attachments/assets/93826e4b-7319-459b-9d84-83929606ef4d" alt="Kinematic-ICP" width="500"/>

</div>

# How to Build

Our system operates on ROS2, supporting **ROS Humble**, **Iron**, and **Jazzy**. To build and run Kinematic-ICP, follow these steps:

1. **Clone the Repository**:
   ```sh
   cd <your_ros_workspace>/src
   git clone https://github.com/PRBonn/kinematic-icp
   cd ..
   ```

2. **Ensure all Dependencies are Installed**:
   ```sh
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace**:
   ```sh
   colcon build
   ```

4. **Source the Setup Script**:
   ```sh
   source ./install/setup.bash
   ```


# TF Requirements

Kinematic ICP can enhance existing odometry using a 3D LiDAR. However, there are specific requirements regarding motion and transformations due to the assumption that the robot operates on a unicycle kinematic model. Below are the key requirements:

1. **Planar Movement**: The robot is expected to move primarily on a planar surface.

2. **Existing Odometry**: An existing odometry source must be provided, such as the platform's wheel odometry. In the ROS ecosystem, this means that another node must publish the `tf` transformation between `base_link` and `odom`. (Note: The names may vary and can be adjusted in the pipeline parameters.)

3. **Static Transform for LiDAR**: To utilize the platform's motion model effectively, the system needs to compute the pose in `base_link`. Therefore, a static `tf` transform between `base_link` and the LiDAR frame (extrinsic calibration) is required. If this calibration is significantly inaccurate, it may compromise system performance.

Finally, Kinematic ICP will publish a new `tf` transformation between `base_link` and `odom_lidar`.

# Running the System

This system offers two entry points for deployment, depending on your use case: one for real-time operation and one for offline processing.

## 1. Real-Time Deployment: `online_node`

Use the `online_node` to run the system on a robotics platform. The only required parameter is the **lidar_topic**. You can start the system using the following command:

```sh
ros2 launch kinematic_icp online_node.launch.py lidar_topic:=<TOPIC>
```

To enable simultaneous visualization through RViz, use the `visualize` flag set to `true`:

```sh
ros2 launch kinematic_icp online_node.launch.py lidar_topic:=<TOPIC> visualize:=true
```

## 2. Offline Processing: `offline_node`

For post-processing and analysis, the `offline_node` processes a ROS bag file at CPU speed, ensuring no frames are dropped. This mode is ideal for reviewing trajectory results, debugging, and speeding up bag file processing. You can launch the offline node with the following command:

```sh
ros2 launch kinematic_icp offline_node.launch.py lidar_topic:=<TOPIC> bag_filename:=<ROSBAG>
```

RViz can also be used in this mode by setting the `visualize` flag to `true`. Additionally, the system will output a file in TUM format containing the estimated poses, named **\<ROSBAG>_kinematic_poses_tum.txt**. This file is saved in the same directory as the ROS bag file by default.

To specify a custom directory for the output file, use the `output_dir` parameter:

```sh
ros2 launch kinematic_icp offline_node.launch.py lidar_topic:=<TOPIC> bag_filename:=<ROSBAG> output_dir:=<OUTPUT_DIRECTORY>
```


## Citation

If you use this library for any academic work, please cite our original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/kissteam2025icra.pdf).

```bibtex
@article{kissteam2024arxiv,
  author    = {Guadagnino ,Tiziano and Mersch, Benedikt and Vizzo, Ignacio and Gupta, Saurabh and Malladi, Meher V.R. and Lobefaro, Luca and Doisy, Guillaume and Stachniss, Cyrill}
  title     = {{Kinematic-ICP: Enhancing LiDAR Odometry with Kinematic Constraints for Wheeled Mobile Robots Moving on Planar Surfaces
  journal   = arXiv Preprint,
  year      = {2024},
  codeurl   = {https://github.com/PRBonn/kinematic-icp},
}
```
