#!/usr/bin/env python3

import cv2
import os
import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from vision_capture2.srv import VisionCapture, VisionCaptureResponse

def cloud_callback(msg):
    global cloud_data
    cloud_data = msg

def LOGGING(state):
    homeDir = os.getenv("HOME")
    if homeDir is None:
        sys.stderr.write("Failed to get the home directory.\n")

    file_loc = 'logfile/logging_file.txt'

    with open(file_loc, 'a') as file:
        if file.tell() != 0:
            file.write('\n')
        file.write(state)

def save_point_cloud_as_pcd():
    global cloud_data
    if cloud_data is not None:
        pc_np = pc2.read_points(cloud_data, field_names=("x", "y", "z"), skip_nans=True)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_np)

        bounding_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[-0.168, -0.1, 0.3],
            max_bound=[0.3, 0.075, 0.4]
        )

        cropped_pcd = pcd.crop(bounding_box)
        
        filtered_pcd = average_z_filter_for_x_direction(cropped_pcd)

        open("files/point_cloud.pcd", "w").close()

        o3d.io.write_point_cloud("files/point_cloud.pcd", filtered_pcd)


def get_boundary( pcd ):
    points = np.asarray(pcd.points)

    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])

    point_cloud = o3d.geometry.PointCloud()
    boundary_cloud = np.array([[min_x, min_y, 30], [min_x, max_y, 30], [max_x, max_y, 30], [max_x, min_y, 30], [min_x, min_y, 30]])
    point_cloud.points = o3d.utility.Vector3dVector(boundary_cloud)
    
    #for p in point_cloud.points:
    #    print("x:", p[0], "y:", p[1], "z:", p[2])

    o3d.io.write_point_cloud("files/boundary_cloud.pcd", point_cloud)

def average_z_filter_for_x_direction(cropped_pcd):
    filtered_pcd = o3d.geometry.PointCloud()
        
    # find the max_x & min_x
    max_x = float('-inf')
    min_x = float('inf')
    for point in cropped_pcd.points:
        x = point[0]
        if x > max_x:
            max_x = x
        if x < min_x:
            min_x = x

    x_resolution = 0.005 # resolution = 1cm
    current_x = min_x

    while current_x < max_x:
        loop_avg_z = 0
        loop_sum_z = 0
        count = 0
        for tmp_points in cropped_pcd.points:
            if tmp_points[0] < current_x + x_resolution and tmp_points[0] >= current_x:
                loop_sum_z += tmp_points[2]
                count += 1

        if count > 0:
            loop_avg_z = loop_sum_z / count

        for tmp_points in cropped_pcd.points:
            if tmp_points[0] < current_x + x_resolution and tmp_points[0] >= current_x:
                tmp_points[2] = loop_avg_z
                
                filtered_pcd.points.append(tmp_points[:3])  # Only keep the x, y, z coordinates

        current_x = current_x + x_resolution

    return filtered_pcd

def capture(req):
    global cloud_data
    cloud_data = None

    if req.scan == True:
        # if msg(subscriber) is not None
        while cloud_data is None:
            rospy.Subscriber("/camera/depth/color/points", PointCloud2, cloud_callback)
        
        save_point_cloud_as_pcd()
        
        cropped_pcd = o3d.io.read_point_cloud("files/point_cloud.pcd")

        get_boundary( cropped_pcd )

        boundary_cloud = o3d.io.read_point_cloud("files/boundary_cloud.pcd")

        # visualize
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=np.zeros(3))
        blue_color = np.array([[0.0, 0.0, 1.0] for _ in range(len(cropped_pcd.points))])
        cropped_pcd.colors = o3d.utility.Vector3dVector(blue_color)
        scene = [cropped_pcd, axes]
        o3d.visualization.draw_geometries(scene, window_name="Cropped Point Cloud with XYZ Axes", width=800, height=600)
        o3d.visualization.draw_geometries([boundary_cloud], window_name="Cropped Point Cloud with XYZ Axes", width=800, height=600)

        return VisionCaptureResponse(True)

if __name__ == '__main__':
    rospy.init_node('vision_capture')
    s = rospy.Service('/vision_capture', VisionCapture, capture)
    rospy.spin()