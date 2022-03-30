#!/usr/bin/env python
import os
import sys
import message_filters
import rospy
import numpy as np
import open3d as o3d

from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__=="__main__":
    rgb_image_folder = input("Enter the absolute path to the folder where the rgb images should be saved or just press enter to save in the current directory: ")
    if rgb_image_folder == "":
        rgb_image_folder = os.path.join(os.getcwd(), "rgb_images/")
        if os.path.isdir(rgb_image_folder) == False:
            os.mkdir(rgb_image_folder)
    elif os.path.isdir(rgb_image_folder) == True:
        pass
    else:
        os.mkdir(rgb_image_folder)

    depth_image_folder = input("Enter the absolute path to the folder where the depth images should be saved or just press enter to save in the current directory: ")
    if depth_image_folder == "":
        depth_image_folder = os.path.join(os.getcwd(), "depth_images/")
        if os.path.isdir(depth_image_folder) == False:
            os.mkdir(depth_image_folder)
    elif os.path.isdir(depth_image_folder) == True:
        pass
    else:
        os.mkdir(depth_image_folder)

    vis = o3d.visualization.Visualizer()
    vis.capture_depth_image("%sdepth_image_test1.png" %(depth_image_folder), do_render=True)