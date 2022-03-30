#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

num = 1
    
def listener():
    global num
    
    input("Press Enter to take an image")
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    msg1 = rospy.wait_for_message("/camera/color/image_raw", Image)
    msg2 = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)
    
    try:
        img1 = CvBridge()
        img1 = img1.imgmsg_to_cv2(msg1, "bgr8")
        img2 = CvBridge()
        img2 = img2.imgmsg_to_cv2(msg2, "8UC1")
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imwrite("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % (num*2-1), img1)
        cv2.imwrite("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % (num*2), img2)
    num += 1

if __name__ == '__main__':
    try:
        while True:
            listener()
    except KeyboardInterrupt:
    	exit()
