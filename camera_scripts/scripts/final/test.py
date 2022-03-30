#!/usr/bin/env python
import cv2
import rospy
import message_filters
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

'''def callback(rgb_msg, depth_msg):
    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
        #cv2.imshow("Visualiser", depth_img)

        plt.subplot(1, 2, 1)
        plt.title('RGB Image')
        plt.imshow(rgb_img)
        plt.subplot(1, 2, 2)
        plt.title('Depth Image')
        plt.imshow(depth_img)
        plt.show()
    except CvBridgeError as e:
        print(e)

rospy.init_node('listener', anonymous=True)

rgb_msg = message_filters.Subscriber('/camera/color/image_raw', Image)
depth_msg = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)

ts = message_filters.TimeSynchronizer([rgb_msg, depth_msg], 10)
ts.registerCallback(callback)

# plt.subplot(1, 2, 1)
# plt.title('RGB Image')
# plt.imshow(rgb_img)
# plt.subplot(1, 2, 2)
# plt.title('Depth Image')
# plt.imshow(depth_img)
# plt.show()

rospy.spin()'''



rospy.init_node('listeners', anonymous=True)

def callback(mode, penalty):
    rgb_img = CvBridge()
    rgb_img = rgb_img.imgmsg_to_cv2(mode, "bgr8")
    depth_img = CvBridge()
    depth_img = depth_img.imgmsg_to_cv2(penalty, "8UC1")
    print(np.shape(rgb_img))
    print(np.shape(depth_img))

    #combined = np.concatenate((rgb_img, depth_img))
    #print(combined)
    #cv2.imshow("OpenCV", np.array(rgb_img, dtype=np.uint8))

    '''plt.subplot(1, 2, 1)
    plt.title('RGB Image')
    plt.imshow(rgb_img)
    plt.subplot(1, 2, 2)
    plt.title('Depth Image')
    plt.imshow(depth_img)
    plt.show()'''

    #return rgb_img, depth_img

# def display_image()

mode_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
penalty_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)

# ts = message_filters.ApproximateTimeSynchronizer([mode_sub, penalty_sub],10, 0.1, allow_headerless=True)
ts = message_filters.TimeSynchronizer([mode_sub, penalty_sub],10)
try:
    print("111111111111111111111111")
    ts.registerCallback(callback)
    print("222222222222222222222222")
except:
    print("Unsynchronised")
'''# print(combined)
splitted = np.array_split(combined, 2)
rgb_img = splitted[0]
print(np.shape(rgb_img))
depth_img = splitted[1]
print(np.shape(depth_img))'''

rospy.spin()

'''a = np.array([[1, 3, 5], [2, 4, 6]])
print(a)
b = np.array([[1, 2, 3], [4, 5, 6]])
print(b)
c = np.concatenate((a, b))
d = np.array_split(c, 2)
x = d[0]
print(x)
y = d[1]
print(y)'''