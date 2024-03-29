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

global image, tag_rotation, tag_translation, centroid

camera_params = (610.319671, 611.570539, 317.992731, 235.564522)
tag_size = 0.0408
tag_rotation = None
tag_translation = None

bridge = CvBridge()
image = None
cloud_data = None
centroid = None

def image_callback(msg):
    global image
    try:
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        pose_rotation, pose_translation = get_pose()
        TF(pose_rotation, pose_translation)
        
    except Exception as e:
        rospy.logerr("Error processing image: %s" % str(e))

def get_pose():
    global image, tag_rotation, tag_translation

    if image is not None:
        at_detector = Detector(families='tag36h11')
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

        if len(tags) == 0:
            print('\033[91m' + " [calibration] Calibration Failed : No APRILTAG detected." + '\033[0m')

            return None, None

        else:
            for tag in tags:
                if 0 == tag.tag_id :
                    tag_rotation = tag.pose_R
                    tag_translation = tag.pose_t
                    print('\033[92m' + " [calibration] Calibration Success" + '\033[0m')

    #TF(tag_rotation, tag_translation)
    return tag_rotation, tag_translation

def TF(rotation, transition):
    print("tf ............")
    homeDir = os.getenv("HOME")
    if homeDir is None:
        sys.stderr.write("Failed to get the home directory.\n")

    if rotation is None or transition is None:
        print("Error: Rotation or translation is None.")
        return

    # file_loc = homeDir + '/PSP/files/TF.txt'
    file_loc = '/home/honglang/PSP/files/TF.txt'

    tf = []
    for i in range(3):
        for j in range(3):
            tf.append(rotation[i][j])
    for i in range(3):
        tf.append(transition[0][i])

    with open(file_loc, 'w') as file:
        for i in range(len(tf)):
            file.write(str(tf[i]))  # Convert to string before writing
            file.write('\n')

def ros_server():
    rospy.init_node('vision_capture')
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    ros_server()
