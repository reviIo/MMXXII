#!/usr/bin/env python
import os
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = [data.width/2, data.height/2]
            [pix[0], pix[1]] = [int(pix[0]), int(pix[1])]
            sys.stdout.write('Topic: %s: Depth at center(%d, %d): %d mm\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    rospy.init_node("listener")
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    rospy.spin()
    print()