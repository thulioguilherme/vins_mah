#!/usr/bin/env python3

"""
Synchronize IMU data and create ROS2 bag with video, Ground Truth, and Namespaces

Description:
    Creates data structure containing IMU and ARKit data, then
    generates a ROS2 bag containing the images, IMU, and ground truth topics
    with customizable namespaces and a fixed 'advio' frame_id prefix.

Copyright (C) 2018 Santiago Cortes (Original ROS1 / Sync Versions)
Fused ROS2 Port Version with Argparse, Namespaces, and Sequential Frame Rate Fix
"""

import os
import argparse
import pandas as pd
import numpy as np
import cv2

# Import ROS2 libraries
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time as MsgTime
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import rosbag2_py


def add_namespace_to_topic(namespace, topic):
    """Prepends a namespace to a topic name securely."""
    if not namespace:
        return topic
    namespace = namespace.rstrip("/")
    if topic.startswith("/"):
        return f"{namespace}{topic}"
    return f"{namespace}/{topic}"


def main():
    parser = argparse.ArgumentParser(
        description="Synchronize IMU data and create a ROS2 bag for the ADVIO dataset."
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to the main data folder containing the advio sequence directories."
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output directory to save the generated ROS2 bags. If not set, saves inside each sequence's iphone folder."
    )
    parser.add_argument(
        "--sequences",
        nargs="*",
        type=int,
        default=list(range(1, 24)),
        help="Specific sequences to bag (e.g. 1 2 3). Defaults to all 1-23."
    )
    parser.add_argument(
        "--namespace",
        default="",
        help="Namespace to prepend to every topic and frame_id (e.g. /uav33)."
    )

    args = parser.parse_args()

    print(f"Creating synchronized ROS2 bags for sequences in {args.sequences}")

    # Delete first frames threshold
    start = 0.5
    bridge = CvBridge()

    # Apply namespaces to the frame_id prefix
    ns = args.namespace.strip("/")
    prefix = f"{ns}/advio_" if ns else "advio_"
    
    cam_frame_id = f"{prefix}cam0"
    imu_frame_id = f"{prefix}imu0"
    world_frame_id = f"advio/world"

    cam_topic = add_namespace_to_topic(args.namespace, "/cam0/image_raw")
    imu_topic = add_namespace_to_topic(args.namespace, "/imu0")
    pose_topic = add_namespace_to_topic(args.namespace, "/groundtruth/odom")

    # Loop through all sequences
    for i in args.sequences:
        seq_str = str(i).zfill(2)
        seq_folder = f"{args.input}/advio-{seq_str}"
        dir_path = f"{seq_folder}/iphone"
        gt_dir_path = f"{seq_folder}/ground-truth"
        
        if not os.path.exists(dir_path):
            print(f"Directory not found: {dir_path}. Skipping.")
            continue

        print(f"\nProcessing directory: {dir_path}")

        # ==========================================
        # PHASE 1: LOAD & PARSE DATA
        # ==========================================
        
        acc_path = f"{dir_path}/accelerometer.csv"
        gyro_path = f"{dir_path}/gyro.csv"
        pose_path = f"{gt_dir_path}/pose.csv"
        arkit_path = f"{dir_path}/arkit.csv"
        
        acc = pd.read_csv(acc_path, names=list('tabc'))
        gyro = pd.read_csv(gyro_path, names=list('tabc'))
        gt_poses = pd.read_csv(pose_path, names=['t', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
        arkit = pd.read_csv(arkit_path, names=list('tabcdefg'))
        
        g = []
        a = []
        
        t = gyro['t'].values.astype(float)
        t_acc = acc['t'].values.astype(float)
        zer = t * 0.0
        
        # Interpolate accelerometer data to match gyroscope timestamps
        for c in 'abc':
            a.append(np.interp(
                t, 
                t_acc, 
                acc[c].values.astype(float)
            ))
            g.append(gyro[c].values.astype(float))
            
        # IMU data block (type ID 34)
        M = np.column_stack((t, zer + 34, g[0], g[1], g[2], a[0], a[1], a[2], zer, zer))
        
        v = []
        t_gt = gt_poses['t'].values.astype(float)
        zer_gt = t_gt * 0
        
        # Ground truth data block (type ID 8)
        for c in ['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']:
            v.append(gt_poses[c].values.astype(float))
            
        M_gt = np.column_stack((t_gt, zer_gt + 8, np.arange(len(zer_gt)), v[0], v[1], v[2], v[3], v[4], v[5], v[6]))
        
        # Combine IMU and Ground Truth into a single sorted timeline
        full = np.concatenate((M, M_gt))
        full = full[full[:, 0].argsort()]

        # ==========================================
        # PHASE 2: CREATE ROS2 BAG
        # ==========================================
        
        if args.output:
            os.makedirs(args.output, exist_ok=True)
            bag_uri = os.path.join(args.output, f"advio_{seq_str}")
        else:
            bag_uri = f"{dir_path}/iphone_bag"

        print(f"Saving ROS2 bag to: {bag_uri}")

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=bag_uri,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py._storage.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        writer.open(storage_options, converter_options)
        
        # Create ROS2 topics
        topic_meta_cam = rosbag2_py._storage.TopicMetadata(
            name=cam_topic, type='sensor_msgs/msg/Image', serialization_format='cdr'
        )
        topic_meta_imu = rosbag2_py._storage.TopicMetadata(
            name=imu_topic, type='sensor_msgs/msg/Imu', serialization_format='cdr'
        )
        topic_meta_pose = rosbag2_py._storage.TopicMetadata(
            name=pose_topic, type='nav_msgs/msg/Odometry', serialization_format='cdr'
        )
        
        writer.create_topic(topic_meta_cam)
        writer.create_topic(topic_meta_imu)
        writer.create_topic(topic_meta_pose)

        # 1. Write Image/Camera messages sequentially using arkit timestamps
        vidcap = cv2.VideoCapture(f"{dir_path}/frames.mov")
        vlength = int(vidcap.get(cv2.CAP_PROP_FRAME_COUNT))
        t_arkit = arkit['t'].values.astype(float)
        
        print(f"Total video frames: {vlength}, Arkit triggers: {len(t_arkit)}")

        success = True
        frame_idx = 0
        while success and frame_idx < len(t_arkit):
            success, image = vidcap.read()
            if not success:
                break

            # if success:
            #     print(f"Actual video frame shape: {image.shape}") # Should show (height, width, channels)       

            stamp = t_arkit[frame_idx]
            if stamp > start:
                sec = int(stamp)
                nanosec = int((stamp - sec) * 1e9)
                msg_time = MsgTime(sec=sec, nanosec=nanosec)
                bag_timestamp_ns = int(stamp * 1e9)

                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # gray_image=cv2.transpose(gray_image)
                # gray_image=cv2.flip(gray_image,+1)

                Img = bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
                Img.header.stamp = msg_time
                Img.header.frame_id = cam_frame_id
                writer.write(cam_topic, serialize_message(Img), bag_timestamp_ns)

            frame_idx += 1
            if frame_idx % 500 == 0:
                print(f"Video Processing: {int(np.round(100 * frame_idx / max(1, vlength)))}%")

        vidcap.release()

        # 2. Write IMU and Ground Truth messages from the sorted timeline
        for row in full:
            stamp = float(row[0])
            row_type = int(row[1])
            
            if stamp <= start:
                continue
                
            sec = int(stamp)
            nanosec = int((stamp - sec) * 1e9)
            msg_time = MsgTime(sec=sec, nanosec=nanosec)
            bag_timestamp_ns = int(stamp * 1e9)

            # Ground Truth Odometry (ID 8)
            if row_type == 8:
                odom = Odometry()
                odom.header.stamp = msg_time
                odom.header.frame_id = world_frame_id
                odom.child_frame_id = imu_frame_id
                
                odom.pose.pose.position.x = float(row[3])
                odom.pose.pose.position.y = float(row[4])
                odom.pose.pose.position.z = float(row[5])
                
                odom.pose.pose.orientation.x = float(row[6])
                odom.pose.pose.orientation.y = float(row[7])
                odom.pose.pose.orientation.z = float(row[8])
                odom.pose.pose.orientation.w = float(row[9])
                
                writer.write(pose_topic, serialize_message(odom), bag_timestamp_ns)
    
            # IMU Measurement (ID 34)
            elif row_type == 34:
                gyr = row[2:5]
                acc = row[5:8]
                
                imu = Imu()
                imu.header.stamp = msg_time
                imu.header.frame_id = imu_frame_id
                
                imu.angular_velocity.x = float(gyr[0])
                imu.angular_velocity.y = float(gyr[1])
                imu.angular_velocity.z = float(gyr[2])

                imu.linear_acceleration.x = float(acc[0])
                imu.linear_acceleration.y = float(acc[1])
                imu.linear_acceleration.z = float(acc[2])
                
                writer.write(imu_topic, serialize_message(imu), bag_timestamp_ns)
                
        # Cleanup memory to ensure SQLite database locks are released 
        del writer

    print('\nProcess complete')


if __name__ == "__main__":
    main()
