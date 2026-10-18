# VINS-Mah

## Changes from the original repository

* Updated the original implementation to ROS 2 (tested on Humble/Ubuntu 22.04).

* Created the `refactor_viode_rosbag.py` script to update the frame_ids and topic names of the VIODE rosbags, making them easy to visualize on Rviz2

```bash
python3 refactor_viode_rosbag.py --input bags/input --output bags/output --namespace /uav33 --rename-topics /odometry=/groundtruth/odom --set-frame-id /imu0=uav33/viode_imu0 /cam0/image_raw=uav33/viode_cam0 /cam0/segmentation=uav33/viode_cam0 /cam1/image_raw=uav33/viode_cam1 /cam1/segmentation=uav33/viode_cam1 /odometry=uav33/viode_world --set-child-frame-id /odometry=uav33/viode_imu0
```

## A visual-inertial odometry for dynamic environments
The VINS-Mah achieves better performance in dynamic environments than the baseline methods while not degrading in static scenarios. It is built on the well-known method [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git).

Original repository README: https://github.com/hyq123-cmd/VINS-Mah
