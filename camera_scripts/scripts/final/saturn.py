#!/usr/bin/env python
import sys
import rospy
import threading
import message_filters
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib.animation import FuncAnimation

'''def rgb_callback(rgb_msg):
    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
        print(1)
    except CvBridgeError as e:
        print(e)

    return rgb_img

def depth_callback(depth_msg):
    try:
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
        print(2)
    except CvBridgeError as e:
        print(e)

    return depth_img'''

'''def get_image_stream(rgb_msg, depth_msg):
    # global vis
    global fig, ax1, ax2
    global frame_num, fps_list

    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
    except CvBridgeError as e:
        print(e)

    start_time = datetime.now()
    depth_img = np.expand_dims(depth_img, axis=-1)

    rgb_img = np.array(rgb_img, dtype=np.uint8)
    depth_img = np.array(depth_img, dtype=np.uint8)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb_img), o3d.geometry.Image(depth_img), convert_rgb_to_intensity = False
    )
    


    # vis.update_geometry(rgbd_image)
    # vis.update_renderer()
    # if not vis.poll_events():
    #     vis.close()
    #     sys.exit()

    
    # vis.clear_geometries()
    # vis.add_geometry(rgbd_image)
    # # o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.35)



    fig = plt.figure("Figure", figsize=(12, 8))
    plt.subplot(1, 2, 1)
    plt.title('RGB Image Stream')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('Depth Image Stream')
    plt.imshow(rgbd_image.depth)
    plt.show(block=False)
    fig.canvas.draw_idle()
    fig.canvas.update()
    fig.canvas.flush_events()

    process_time = datetime.now() - start_time
    print("Frame Number %d" % frame_num)
    fps = 1/process_time.total_seconds()
    print("FPS = %d" % (fps))
    fps_list.append(fps)

    frame_num += 1


if __name__=="__main__":
    # vis = o3d.visualization.Visualizer()
    # vis.create_window("Open3D Viewer")

    frame_num = 1
    fps_list = []

    try:
        # fig = plt.figure()
        # ax1 = fig.add_subplot(1,2,1)
        # ax2 = fig.add_subplot(1,2,2)
        while True:
            rospy.init_node('listener', anonymous=True)

            # rgb_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
            # depth_msg = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)

            # ats = message_filters.ApproximateTimeSynchronizer([rgb_msg, depth_msg], 10, 0.1, allow_headerless=True)
            # ats.registerCallback(get_image_stream)

            rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
            depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)

            get_image_stream(rgb_msg, depth_msg)

            # plt.show()

            # rospy.spin()
    except KeyboardInterrupt:
        print("The average fps is %.3f" % (sum(fps_list[:-1])/(len(fps_list)-1)))
        sys.exit()'''








'''fig = plt.figure()
ax1 = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2)

def animate(i):
    rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)

    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
    except CvBridgeError as e:
        print(e)
    
    xar = []
    yar = []
    for eachLine in depth_img:
        if len(eachLine)>1:
            xar.append(int(x))
            yar.append(int(y))
    ax1.clear()
    ax1.plot(xar,yar)
ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()'''







'''import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import psutil
import collections

cpu = collections.deque(np.zeros(10))
ram = collections.deque(np.zeros(10))

print("CPU: {}".format(cpu))
print("Memory: {}".format(ram))

def my_function():
    cpu.popleft()
    cpu.append(psutil.cpu_percent(interval=1))
    
    ram.popleft()
    ram.append(psutil.virtual_memory().percent)

cpu = collections.deque(np.zeros(10))
ram = collections.deque(np.zeros(10))

# test
my_function()
my_function()
my_function()

print("CPU: {}".format(cpu))
print("Memory: {}".format(ram))

# function to update the data
def my_function(i):
    # get data
    cpu.popleft()
    cpu.append(psutil.cpu_percent())
    ram.popleft()
    ram.append(psutil.virtual_memory().percent)
    
    # clear axis
    ax.cla()
    ax1.cla()
    
    # plot cpu
    ax.plot(cpu)
    ax.scatter(len(cpu)-1, cpu[-1])
    ax.text(len(cpu)-1, cpu[-1]+2, "{}%".format(cpu[-1]))
    ax.set_ylim(0,100)
    
    # plot memory
    ax1.plot(ram)
    ax1.scatter(len(ram)-1, ram[-1])
    ax1.text(len(ram)-1, ram[-1]+2, "{}%".format(ram[-1]))
    ax1.set_ylim(0,100)

# start collections with zeros
cpu = collections.deque(np.zeros(10))
ram = collections.deque(np.zeros(10))

# define and adjust figure
fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
ax = plt.subplot(121)
ax1 = plt.subplot(122)
ax.set_facecolor('#DEDEDE')
ax1.set_facecolor('#DEDEDE')

# animate
ani = FuncAnimation(fig, my_function, interval=1000)

plt.show()'''











'''import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import psutil
import collections

# # test
# my_function()
# my_function()
# my_function()

# print("CPU: {}".format(cpu))
# print("Memory: {}".format(ram))

# function to update the data
def my_function(i):
    rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    depth_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)

    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
    except CvBridgeError as e:
        print(e)
    
    # clear axis
    plt.cla()
    plt.cla()
    
    # plot cpu
    plt.plot(rgb_img)
    # ax1.scatter(len(cpu)-1, cpu[-1])
    # ax1.text(len(cpu)-1, cpu[-1]+2, "{}%".format(cpu[-1]))
    # ax1.set_ylim(0,100)
    
    # plot memory
    plt.plot(depth_img)
    # ax2.scatter(len(ram)-1, ram[-1])
    # ax2.text(len(ram)-1, ram[-1]+2, "{}%".format(ram[-1]))
    # ax2.set_ylim(0,100)

# define and adjust figure
fig = plt.figure(figsize=(12,6))
ax1 = plt.subplot(121)
ax2 = plt.subplot(122)

# animate
ani = FuncAnimation(fig, my_function, interval=1000)

plt.show()'''












frame_num = 1
fps_list = []

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









'''import cv2
import matplotlib.pyplot as plt

# def grab_frame(cap):
#     ret,frame = cap.read()
#     return cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

def grab_rgb_frame():
    cap = rospy.wait_for_message("/camera/color/image_raw", Image)

    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(cap, "rgb8")
        cap = rgb_img
    except CvBridgeError as e:
        print(e)

    return cap

def grab_depth_frame():
    cap = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)

    try:
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(cap, "8UC1")
        cap = depth_img
    except CvBridgeError as e:
        print(e)
    
    return cap

def close(event):
    if event.key == 'q':
        plt.close(event.canvas.figure)

rospy.init_node("gossiper", anonymous=True)

frame_num = 1
fps_list = []

#create two subplots
ax1 = plt.subplot(1,2,1)
ax2 = plt.subplot(1,2,2)

#create two image plots
im1 = ax1.imshow(grab_rgb_frame())
im2 = ax2.imshow(grab_depth_frame())

plt.ion()

while True:
    start_time = datetime.now()

    im1.set_data(grab_rgb_frame())
    im2.set_data(grab_depth_frame())
    plt.pause(0.01)

    process_time = datetime.now() - start_time
    print("Frame Number %d" % frame_num)
    fps = 1/process_time.total_seconds()
    print("FPS = %d" % (fps))
    fps_list.append(fps)

    frame_num += 1

    #cid = plt.gcf().canvas.mpl_connect("key_press_event", close)

plt.ioff() # due to infinite loop, this gets never called.
plt.show()'''