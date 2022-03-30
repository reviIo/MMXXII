#!/usr/bin/env python
import os
import sys
import cv2
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
            quit()
        else:
            print("Error in user input.")
            input_fn(rgb_image_folder, depth_image_folder)

class subscribing():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.rgb_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_msg = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        self.ats = message_filters.ApproximateTimeSynchronizer([self.rgb_msg, self.depth_msg], 10, 0.1, allow_headerless=True)

        rospy.spin()

    def rgb_callback(self):
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(self.rgb_msg, "rgb8")

        return rgb_img

    def depth_callback(self):
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(self.depth_msg, "8UC1")

        return depth_img

    def callback(self):
        print(type(self.rgb_msg))
        print(type(self.depth_msg))

        try:
            rgb_img = self.rgb_callback()
            depth_img = self.depth_callback()
        except CvBridgeError as e:
            print(e)

        return rgb_img, depth_img

###############################################################################################################

# Capturing a RGB and depth image and viewing it (from subscriber2.py)
    
class image_capture():
    def __init__(self, rgb_image_folder, depth_image_folder):
        self.rgb_image_path = rgb_image_folder
        self.depth_image_path = depth_image_folder
    
    def get_rgb_and_depth_images(self, rgb_img, depth_img):
        global num

        cv2.imwrite("%srgb_image%s.png" % (rgb_image_folder, num), rgb_img)
        cv2.imwrite("%sdepth_image%s.png" % (depth_image_folder, num), depth_img)

        '''plt.subplot(1, 2, 1)
        plt.title('RGB Image')
        plt.imshow(rgb_img)
        plt.subplot(1, 2, 2)
        plt.title('Depth Image')
        plt.imshow(depth_img)
        plt.show()'''

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

        print(rgb_image_folder)
        print(depth_image_folder)

        color = o3d.io.read_image("%srgb_image%s.png" % (rgb_image_folder, num))
        depth = o3d.io.read_image("%sdepth_image%s.png" % (depth_image_folder, num))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = True)
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, camera_intrinsic)
        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        #o3d.visualization.ViewControl.set_front(vis.get_view_control(), [0.9288, -0.2951, -0.2242])
        #o3d.visualization.ViewControl.set_lookat(vis.get_view_control(), [1.6784, 2.0612, 1.4451])
        #o3d.visualization.ViewControl.set_up(vis.get_view_control(), [-0.3402, -0.9189, -0.1996])
        o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.45)
        vis.run()

        user_input = input("Do you wish to save that pointcloud (y/n): ").lower()

        return user_input, pcd

    def save_pointcloud(self, pointcloud):
        # Need to fix it so that the user doesn't have to enter the folder path everytime the function gets called
        pointcloud_folder = input("Enter the absolute path to the folder where the pointclouds should be saved or just press enter to save in the current directory: ")
        if pointcloud_folder == "":
            pointcloud_folder = os.path.join(os.getcwd(), "pointclouds/")
        elif os.path.isdir(pointcloud_folder) == True:
            pass
        else:
            os.mkdir(pointcloud_folder)
        
        # Add functionality to determine the suffix number
        o3d.io.write_point_cloud("%spointcloud%s.pcd" % (pointcloud_folder, num), pointcloud)

    def get_pointcloud(self):
        user_input, pointcloud = self.pointcloud_generation(rgb_image_folder, depth_image_folder)
        if user_input == "y":
            self.save_pointcloud(pointcloud)
        else:
            pass

###############################################################################################################

# Stream the RGB and depth image in real time (from bagreader.py)
# Would be even better if can add a gui so that it would print out the distance for the position that the mouse clicked or likewise

class image_stream():
    def __init__(self):
        self.frame_num = 1
        self.fps_list = []
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("Open3D Viewer")

    def get_image_stream(self, rgb_img, depth_img):
        depth_img = np.expand_dims(depth_img, axis=-1)

        while True:
            start_time = datetime.now()

            #rgb = np.array(rgb_img)
            #depth = np.array(depth_img)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(rgb_img), o3d.geometry.Image(depth_img), convert_rgb_to_intensity = False
            )
            
            self.vis.update_geometry(rgbd_image)
            if not self.vis.poll_events():
                # Note that this should only terminate its own thread
                break
            self.vis.update_renderer()

            process_time = datetime.now() - start_time
            print("Frame Number %d" % self.frame_num)
            fps = 1/process_time.total_seconds()
            print("FPS = %d" % (fps))
            self.fps_list.append(fps)

            self.vis.clear_geometries()
            self.vis.add_geometry(rgbd_image)
            o3d.visualization.ViewControl.set_zoom(self.vis.get_view_control(), 0.35)

            self.frame_num += 1

            # return self.fps_list
        
        self.vis.close()
        
# Need to fix that the line below won't be executed

# print("The average fps is %.3f" % (sum(self.fps_list[:-1])/(len(self.fps_list)-1)))

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
    
    subs = subscribing()
    rgb_img, depth_img = subs.callback()

    img_stream = image_stream()
    thread1 = threading.Thread(target=img_stream.get_image_stream, args=(rgb_img, depth_img))
    thread2 = threading.Thread(target=input_fn, args=(rgb_img, depth_img, rgb_image_folder, depth_image_folder))
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()

    try:
        while True:
            subs = subscribing()
            rgb_img, depth_img = subs.callback()
    except KeyboardInterrupt:
        exit()