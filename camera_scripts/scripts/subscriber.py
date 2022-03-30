#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

num = 0

def img_callback(msg):
    global num
    
    try:
    	img1 = CvBridge()
    	print("Image Received")
    	input("Press Enter to take an image")
    	img2 = img1.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
    	print(e)
    else:
    	cv2.imwrite("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % num, img2)
    num += 1
    	
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("/camera/color/image_raw", Image, img_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
