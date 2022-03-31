#!/usr/bin/env python

################## The FPS averages around 15 atm, needs to be fine-tuned ##################
######## Discontinued, easier to use Rviz or other visualisation softwares instead #########

import os
import sys

import cv2
import rospy
import signal
import subprocess
import matplotlib.pyplot as plt

from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib.animation import FuncAnimation

# Variable assignment
frame_num = 1
fps_list = []
bagfile_folder = os.path.join(os.getcwd(), "recording/")

print(os.listdir(bagfile_folder))

try:
    bag_filename = input("Enter the name of the bag file (please include .bag at the end): ")
    while os.path.isfile("%s%s" %(bagfile_folder, bag_filename))==False:
        print("Invalid filename.")
        bag_filename = input("Enter the name of the bag file (please include .bag at the end): ")
except EOFError as e:
    print(e)
    bag_filename = "two_in_one.bag"

full_path = "%s%s" %(bagfile_folder, bag_filename)

# Start a process which plays the targeted rosbag file and initiate the ROS node
command = subprocess.Popen(['rosbag', 'play', full_path], cwd=os.getcwd())
rospy.init_node("eavesdropper", anonymous=True)

# Retrieve the rgb image and convert it to a useful format
def grab_rgb_frame():
    rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)

    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
    except CvBridgeError as e:
        print(e)

    return rgb_img

# Retrieve the depth image and convert it to a useful format
def grab_depth_frame():
    depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)

    try:
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
    except CvBridgeError as e:
        print(e)
    
    return depth_img

# Create two subplots
fig = plt.figure(figsize=(12,8))
ax1 = plt.subplot(1,2,1)
ax2 = plt.subplot(1,2,2)

# Create two image plots
rgb_img = ax1.imshow(grab_rgb_frame())
depth_img = ax2.imshow(grab_depth_frame())

# Grab new frame and calculate the time used
def update(i):
    global frame_num, fps_list

    start_time = datetime.now()

    rgb_img.set_data(grab_rgb_frame())
    depth_img.set_data(grab_depth_frame())

    process_time = datetime.now() - start_time
    print("Frame Number %d" % frame_num)
    fps = 1/process_time.total_seconds()
    print("FPS = %d" % (fps))
    fps_list.append(fps)

    frame_num += 1

# Replace current frame with the new one
ani = FuncAnimation(fig, update, interval=0)

# Close the window if the key "q" is pressed
def close(event):
    if event.key == 'q':
        plt.close(event.canvas.figure)

cid = fig.canvas.mpl_connect("key_press_event", close)

plt.show()

# Kill the process that is playing the rosbag file and exit the program
os.killpg(os.getpgid(command.pid), signal.SIGTERM)
