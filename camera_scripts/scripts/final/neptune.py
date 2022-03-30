import os
import sys
import rospy
import signal
import subprocess
import matplotlib.pyplot as plt

from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib.animation import FuncAnimation

frame_num = 1
fps_list = []

bagfile_folder = os.path.join(os.getcwd(), "recording/")

print(os.listdir(bagfile_folder))

bag_filename = input("Enter the name of the bag file (please include .bag at the end): ")

while os.path.isfile("%s%s" %(bagfile_folder, bag_filename))==False:
    print("Invalid filename.")
    bag_filename = input("Enter the name of the bag file (please include .bag at the end): ")

full_path = "%s%s" %(bagfile_folder, bag_filename)

command = subprocess.Popen(['rosbag', 'play', full_path], cwd=os.getcwd())

rospy.init_node("gossiper", anonymous=True)

def grab_rgb_frame():
    rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)

    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
    except CvBridgeError as e:
        print(e)

    return rgb_img

def grab_depth_frame():
    depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)

    try:
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
    except CvBridgeError as e:
        print(e)
    
    return depth_img

#create two subplots
ax1 = plt.subplot(1,2,1)
ax2 = plt.subplot(1,2,2)

#create two image plots
rgb_img = ax1.imshow(grab_rgb_frame())
depth_img = ax2.imshow(grab_depth_frame())

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
    
ani = FuncAnimation(plt.gcf(), update, interval=1)

def close(event):
    if event.key == 'q':
        plt.close(event.canvas.figure)

cid = plt.gcf().canvas.mpl_connect("key_press_event", close)

plt.show()

print("xyz")

os.killpg(os.getpgid(command.pid), signal.SIGTERM)