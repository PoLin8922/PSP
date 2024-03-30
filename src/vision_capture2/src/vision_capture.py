#!/usr/bin/env python3

import cv2
import os
import sys
import numpy as np
from pupil_apriltags import Detector
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from vision_capture2.srv import VisionCapture, VisionCaptureResponse

global centroid

bridge = CvBridge()
image = None
cloud_data = None
centroid = None

def cloud_callback(msg):
    global cloud_data
    cloud_data = msg

def LOGGING(state):
    homeDir = os.getenv("HOME")
    if homeDir is None:
        sys.stderr.write("Failed to get the home directory.\n")

    # file_loc = homeDir + '/PSP/files/logging_file.txt'
    file_loc = '/home/honglang/PSP/logfile/logging_file.txt'

    with open(file_loc, 'a') as file:
        if file.tell() != 0:
            file.write('\n')
        file.write(state)

def center_point_cloud(point_cloud):
    global centroid
    print(centroid)
    centroid = np.mean(np.asarray(point_cloud.points), axis=0)
    translated_pcd = point_cloud.translate(-centroid)
    return translated_pcd

def save_point_cloud_as_pcd():
    global cloud_data
    if cloud_data is not None:
        pc_np = pc2.read_points(cloud_data, field_names=("x", "y", "z"), skip_nans=True)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_np)

        bounding_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[-0.168, -0.1, 0.3],
            max_bound=[0.3, 0.075, 0.43]
        )

        cropped_pcd = pcd.crop(bounding_box)

        downsampled_pcd = cropped_pcd.voxel_down_sample(voxel_size=0.005)

        centroid_pcd = center_point_cloud(downsampled_pcd)

        # find the max_x & min_x
        max_x = float('-inf')
        min_x = float('inf')
        for point in centroid_pcd.points:
            x = point[0]  # 第一个元素是 X 座标
            if x > max_x:
                max_x = x
            if x < min_x:
                min_x = x

        x_resolution = 0.005 # resolution = 1cm
        current_x = min_x

        while current_x < max_x:
            loop_min_x = 100
            for tmp_points in centroid_pcd.points:
                if tmp_points[0] < current_x + x_resolution and tmp_points[0] >= current_x:
                    if tmp_points[2] < loop_min_x:
                        loop_min_x = tmp_points[2]
            for tmp_points in centroid_pcd.points:
                if tmp_points[0] < current_x + x_resolution and tmp_points[0] >= current_x:
                    tmp_points[2] = loop_min_x 

            current_x = current_x + x_resolution



        # o3d.io.write_point_cloud( homeDir + "/PSP/files/point_cloud.pcd", centroid_pcd )
        o3d.io.write_point_cloud("/home/honglang/PSP/files/point_cloud.pcd", centroid_pcd)

def capture(req):
    if req.scan == True:
        save_point_cloud_as_pcd()
        return VisionCaptureResponse(True)

def ros_server():
    rospy.init_node('vision_capture')
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, cloud_callback)
    s = rospy.Service('/vision_capture', VisionCapture, capture)
    rospy.spin()

if __name__ == '__main__':
    ros_server()
