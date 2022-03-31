#!/usr/bin/env python
import os
import sys
import cv2
import rospy
import message_filters

import open3d as o3d
import matplotlib.pyplot as plt

from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

###############################################################################################################

# Variable assignment

num = 1
version = 0
first_pc = True
pointcloud_folder = ""

# Get user input

def input_fn(rgb_img, depth_img, rgb_image_folder, depth_image_folder):
    global num, version

    while True:
        user_input = input("Press enter to capture the set of RGB and depth image or \n\
    Type rt to retake the image or \n\
    Type pc to generate a pointcloud or \n\
    Type exit to exit the program: "
        ).lower()

        while user_input != "" and user_input != "rt" and user_input != "pc" and user_input != "exit":
            print("Invalid Input.")
            user_input = input("Press enter to capture the set of RGB and depth image or \n\
    Type rt to retake the image or \n\
    Type pc to generate a pointcloud or \n\
    Type exit to exit the program: "
        ).lower()

        if user_input == "":
            version = 0
            img_capture = image_capture(rgb_image_folder, depth_image_folder)
            img_capture.get_rgb_and_depth_images(rgb_img, depth_img)
        elif user_input == "rt":
            if num != 1:
                num -= 1
                version += 1
                img_capture = image_capture(rgb_image_folder, depth_image_folder)
                img_capture.get_rgb_and_depth_images(rgb_img, depth_img)
            else:
                print("Invalid action.")
                input_fn(rgb_img, depth_img, rgb_image_folder, depth_image_folder)
        elif user_input == "pc":
            pc_construction = pointcloud_construction(rgb_image_folder, depth_image_folder)
            pc_construction.get_pointcloud()
        elif user_input == "exit":
            sys.exit()
        else:
            print("Error in user input.")
            input_fn(rgb_img, depth_img, rgb_image_folder, depth_image_folder)

# Convert image messages to OpenCV format

def rgb_msg_to_cv(rgb_msg):
    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "bgr8")
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

###############################################################################################################

# Capturing a RGB and depth image and viewing it (from subscriber2.py)
    
class image_capture():
    def __init__(self, rgb_image_folder, depth_image_folder):
        self.sync = False
        self.rgb_image_path = rgb_image_folder
        self.depth_image_path = depth_image_folder
    
    def get_rgb_and_depth_images(self, rgb_img, depth_img):
        global num, version

        # Topics synchronisation

        rospy.init_node('listener', anonymous=True)

        rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
        depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)
        '''if rgb_msg.header.stamp == depth_msg.header.stamp:
            self.sync = True

        while self.sync != True:
            rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
            depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)
            if rgb_msg.header.stamp == depth_msg.header.stamp:
                self.sync = True'''
        
        rgb_img = rgb_msg_to_cv(rgb_msg)
        depth_img = depth_msg_to_cv(depth_msg)

        cv2.imwrite("%srgb_image%s.png" % (self.rgb_image_path, num), rgb_img)
        cv2.imwrite("%sdepth_image%s.png" % (self.depth_image_path, num), depth_img)

        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)

        if num == 1 and version == 0:
            pass
        elif num == 1 and version != 0:
            plt.close("Figure %s.%s"%(num, version-1))
        elif num != 1 and version == 0:
            plt.close("Figure %s.%s"%(num-1, version))
        else:
            plt.close("Figure %s.%s"%(num-1, version-1))
        plt.figure("Figure %s.%s"%(num, version), figsize=(12, 8))
        plt.subplot(1, 2, 1)
        plt.title('RGB Image %s' %(num))
        plt.imshow(rgb_img)
        plt.subplot(1, 2, 2)
        plt.title('Depth Image %s' %(num))
        plt.imshow(depth_img)
        plt.show(block=False)

        num += 1

###############################################################################################################

# Construct the static pointcloud from RGB and depth images (from pointcloud5.py)
# Save the static pointcloud into a bag file, ply file or pcd file

class pointcloud_construction():
    def __init__(self, rgb_image, depth_image):
        self.rgb_image = rgb_image
        self.depth_image = depth_image
    
    def pointcloud_generation(self, rgb_image_folder, depth_image_folder):
        vis = o3d.visualization.Visualizer()
        vis.create_window("Pointcloud Visualisation")

        if num == 1:
            color = o3d.io.read_image("%srgb_image%s.png" % (rgb_image_folder, num))
            depth = o3d.io.read_image("%sdepth_image%s.png" % (depth_image_folder, num))
        else:
            color = o3d.io.read_image("%srgb_image%s.png" % (rgb_image_folder, num-1))
            depth = o3d.io.read_image("%sdepth_image%s.png" % (depth_image_folder, num-1))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)
        
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, camera_intrinsic)
        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        vis.add_geometry(pcd)
        
        ctr = vis.get_view_control()
        '''ctr.set_front(vis.get_view_control(), [0.9288, -0.2951, -0.2242])
        ctr.set_lookat(vis.get_view_control(), [1.6784, 2.0612, 1.4451])
        ctr.set_up(vis.get_view_control(), [-0.3402, -0.9189, -0.1996])'''
        ctr.set_zoom(0.45)
        vis.run()

        while True:
            vis.update_geometry(pcd)
            vis.update_renderer()
            if not vis.poll_events():
                break

        vis.destroy_window()

        user_input = input("Do you wish to save that pointcloud (y/n): ").lower()

        return user_input, pcd

    def save_pointcloud(self, pointcloud):
        global first_pc, pointcloud_folder

        if first_pc == True:
            pointcloud_folder = input("Enter the absolute path to the folder where the pointclouds should be saved or just press enter to save in the current directory: ")
            if pointcloud_folder == "":
                pointcloud_folder = os.path.join(os.getcwd(), "pointclouds/")
            
            if os.path.isdir(pointcloud_folder) == True:
                pass
            else:
                os.mkdir(pointcloud_folder)
            
            first_pc = False
        
        o3d.io.write_point_cloud("%spointcloud%s.pcd" % (pointcloud_folder, num-1), pointcloud)

    def get_pointcloud(self):
        user_input, pointcloud = self.pointcloud_generation(rgb_image_folder, depth_image_folder)
        if user_input == "y" or user_input == "yes":
            self.save_pointcloud(pointcloud)

###############################################################################################################

# Get destination folders, initialise ROS node and run the main code

if __name__ == "__main__":
    rgb_image_folder = input("Enter the absolute path or just press enter to save the rgb images in the current directory: ")
    if rgb_image_folder == "":
        rgb_image_folder = os.path.join(os.getcwd(), "rgb_images/")
        if os.path.isdir(rgb_image_folder) == False:
            os.mkdir(rgb_image_folder)
    elif os.path.isdir(rgb_image_folder) == True:
        pass
    else:
        os.mkdir(rgb_image_folder)

    depth_image_folder = input("Enter the absolute path or just press enter to save the depth images in the current directory: ")
    if depth_image_folder == "":
        depth_image_folder = os.path.join(os.getcwd(), "depth_images/")
        if os.path.isdir(depth_image_folder) == False:
            os.mkdir(depth_image_folder)
    elif os.path.isdir(depth_image_folder) == True:
        pass
    else:
        os.mkdir(depth_image_folder)
    
    try:
        rospy.init_node('listener', anonymous=True)

        rgb_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_msg = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)

        ats = message_filters.ApproximateTimeSynchronizer([rgb_msg, depth_msg], 10, 0.1, allow_headerless=True)

        rgb_img = rgb_msg.registerCallback(rgb_msg_to_cv)
        depth_img = depth_msg.registerCallback(depth_msg_to_cv)

        input_fn(rgb_img, depth_img, rgb_image_folder, depth_image_folder)

    except KeyboardInterrupt:
        sys.exit()
