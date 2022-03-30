#!/usr/bin/env python
import os
import sys
import cv2
from genpy import message
import rospy
import rosbag
import ctypes
import struct
import threading
import message_filters

import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2

from PIL import Image
from datetime import datetime
from resizeimage import resizeimage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2

#import bagpy
#from bagpy import bagreader

# Admin

num = 1

def input_fn(rgb_img, depth_img, rgb_image_folder, depth_image_folder):
    while True:
        user_input = input("Press enter to capture the set of RGB and depth image or type pointcloud to generate a pointcloud or type exit to exit the program: ").lower()

        while user_input != "" and user_input != "pointcloud" and user_input != "exit":
            print("Invalid Input.")
            user_input = input("Press enter to capture the set of RGB and depth image or type pointcloud to generate a pointcloud or type exit to exit the program: ").lower()

        if user_input == "":
            img_capture = image_capture(rgb_image_folder, depth_image_folder)
            img_capture.get_rgb_and_depth_images(rgb_img, depth_img)
        elif user_input == "pointcloud":
            pc_construction = pointcloud_construction(rgb_image_folder, depth_image_folder)
            pc_construction.get_pointcloud()
        elif user_input == "exit":
            sys.exit()
        else:
            print("Error in user input.")
            input_fn(rgb_image_folder, depth_image_folder)

###############################################################################################################

# Utilities

def rgb_msg_to_cv(rgb_msg):
    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
    except CvBridgeError as e:
        print(e)

    return rgb_img

def depth_msg_to_cv(depth_msg):
    try:
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
    except CvBridgeError as e:
        print(e)

    return depth_img

def callback(rgb_msg, depth_msg):
    assert rgb_msg.header.stamp == depth_msg.header.stamp
    print ("Got synchronised messages")

def rgb_callback(rgb_msg):
    rgb_timestamp = rgb_msg.header.stamp

    return rgb_msg, rgb_timestamp

def depth_callback(depth_msg):
    depth_timestamp = depth_msg.header.stamp

    return depth_msg, depth_timestamp

###############################################################################################################

# Capturing a RGB and depth image and viewing it (from subscriber2.py)
    
class image_capture():
    def __init__(self, rgb_image_folder, depth_image_folder):
        self.sync = False
        self.rgb_image_path = rgb_image_folder
        self.depth_image_path = depth_image_folder
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window("Image Viewer")
    
    def get_rgb_and_depth_images(self, rgb_img, depth_img):
        global num

        # Topics synchronisation

        rospy.init_node('listener', anonymous=True)

        rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
        depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)
        if rgb_msg.header.stamp == depth_msg.header.stamp:
            self.sync = True

        while self.sync != True:
            rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
            depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)
            if rgb_msg.header.stamp == depth_msg.header.stamp:
                self.sync = True
        
        rgb_img = rgb_msg_to_cv(rgb_msg)
        depth_img = depth_msg_to_cv(depth_msg)

        cv2.imwrite("%srgb_image%s.png" % (rgb_image_folder, num), rgb_img)
        cv2.imwrite("%sdepth_image%s.png" % (depth_image_folder, num), depth_img)

        '''for i in range(10):
            plt.subplot(1, 2, 1)
            plt.title('RGB Image %s' %(num))
            plt.imshow(rgb_img)
            plt.subplot(1, 2, 2)
            plt.title('Depth Image %s' %(num))
            plt.imshow(depth_img)
            plt.pause(0.05)
        plt.show(block=False)'''

        plt.figure(num, figsize=(12, 8))
        plt.subplot(1, 2, 1)
        plt.title('RGB Image %s' %(num))
        plt.imshow(rgb_img)
        plt.subplot(1, 2, 2)
        plt.title('Depth Image %s' %(num))
        plt.imshow(depth_img)
        plt.show(block=False)

        '''while True:
            rgb_img = np.array(rgb_img)
            depth_img = np.array(depth_img)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(rgb_img), o3d.geometry.Image(depth_img), convert_rgb_to_intensity = False
            )

            self.vis.update_geometry(rgbd_image)
            if not self.vis.poll_events():
                break
            self.vis.update_renderer()

            self.vis.clear_geometries()
            self.vis.add_geometry(rgbd_image)
            # o3d.visualization.ViewControl.set_zoom(self.vis.get_view_control(), 0.35)'''

        num += 1

###############################################################################################################

# Construct the static pointcloud from RGB and depth images (from pointcloud5.py)
# Save the static pointcloud into a bag file or ply file or pcd file

class pointcloud_construction():
    def __init__(self, rgb_image, depth_image):
        self.rgb_image = rgb_image
        self.depth_image = depth_image
    
    def pointcloud_generation(self, rgb_image_folder, depth_image_folder):
        # Need to sort out this part to stop saving file everytime to view a pointcloud

        vis = o3d.visualization.Visualizer()
        vis.create_window("Pointcloud Visualisation")

        color = o3d.io.read_image("%srgb_image%s.png" % (rgb_image_folder, num-1))
        depth = o3d.io.read_image("%sdepth_image%s.png" % (depth_image_folder, num-1))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = True)
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, camera_intrinsic)
        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        
        #vis.add_geometry(pcd)
        #o3d.visualization.ViewControl.set_front(vis.get_view_control(), [0.9288, -0.2951, -0.2242])
        #o3d.visualization.ViewControl.set_lookat(vis.get_view_control(), [1.6784, 2.0612, 1.4451])
        #o3d.visualization.ViewControl.set_up(vis.get_view_control(), [-0.3402, -0.9189, -0.1996])
        #vis.run()

        i=0

        while True:
            vis.update_geometry(pcd)
            if not vis.poll_events():
                break
            vis.update_renderer()
            vis.clear_geometries()
            vis.add_geometry(pcd)
            o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.45)
            print(i)
            i+=1

        vis.close()

        user_input = input("Do you wish to save that pointcloud (y/n): ").lower()

        return user_input, pcd

    def save_pointcloud(self, pointcloud):
        # Need to fix it so that the user doesn't have to enter the folder path everytime the function gets called
        pointcloud_folder = input("Enter the absolute path to the folder where the pointclouds should be saved or just press enter to save in the current directory: ")
        if pointcloud_folder == "":
            pointcloud_folder = os.path.join(os.getcwd(), "pointclouds/")
        
        if os.path.isdir(pointcloud_folder) == True:
            pass
        else:
            os.mkdir(pointcloud_folder)
        
        o3d.io.write_point_cloud("%spointcloud%s.pcd" % (pointcloud_folder, num-1), pointcloud)

    def get_pointcloud(self):
        user_input, pointcloud = self.pointcloud_generation(rgb_image_folder, depth_image_folder)
        if user_input == "y" or user_input == "yes":
            self.save_pointcloud(pointcloud)
        else:
            pass

###############################################################################################################

# Get the distance of a point on the depth image or pointcloud

class get_distance_from_point():
    def __init__(self, depth_image):
        self.depth_image = depth_image

    def get_distance(self):
        depth_array = np.array(self.depth_image)

###############################################################################################################

# The main function

if __name__ == "__main__":
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
    
    print(rgb_image_folder)
    print(depth_image_folder)
    
    try:
        rospy.init_node('listener', anonymous=True)
    
        '''rgb_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_msg = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)

        message_filters.ApproximateTimeSynchronizer([rgb_msg, depth_msg], 10, 0.1, allow_headerless=True)'''

        # rgb_msg = rospy.Subscriber("/camera/color/image_raw", Image, callback)
        # depth_msg = rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)

        rgb_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_msg = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)

        ats = message_filters.ApproximateTimeSynchronizer([rgb_msg, depth_msg], 10, 0.1, allow_headerless=True)

        rgb_img = rgb_msg.registerCallback(rgb_msg_to_cv)
        depth_img = depth_msg.registerCallback(depth_msg_to_cv)

        '''try:
            rgb_img = CvBridge()
            rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
            depth_img = CvBridge()
            depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
        except CvBridgeError as e:
            print(e)'''

        '''pc_construction = pointcloud_construction(rgb_img, depth_img)
        thread1 = threading.Thread(target=input_fn, args=(rgb_img, depth_img, rgb_image_folder, depth_image_folder))
        thread2 = threading.Thread(target=pc_construction.get_pointcloud())
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()'''

        input_fn(rgb_img, depth_img, rgb_image_folder, depth_image_folder)

    except KeyboardInterrupt:
        sys.exit()