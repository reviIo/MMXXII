#!/usr/bin/env python
import os
import sys
import rospy
import scipy.misc
import numpy as np
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image

class ImageListener:
    def __init__(self, topic):
        """Set the constructor for the class"""
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        """Continuous print out the distance of the object at the centre pixel,
        can be altered to find the distance at all pixels within the image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = [data.width/2, data.height/2]
            [pix[0], pix[1]] = [int(pix[0]), int(pix[1])]
            sys.stdout.write('Topic: %s: Depth at center(%d, %d): %d mm\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return

    # Below is an example of the same function as above, but saves the distance 
    # of all the pixels within the image into a csv file. The function can be modified
    # again to calculate the size of the melon if corner pixels of a bounding box is 
    # found with the color image and depth image mapped accordingly
    """def imageDepthCallback(self, data):
        height = 480
        width = 640
        matrix = np.zeros((height,width))

        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        scipy.misc.imsave("/home/aspa2/rockmelon/MMXXII/mask_rcnn/tools/record/corr_img.png", cv_image)

        try:
            for v in range(height):
                for u in range(width):
                    pix = [v, u]
                    matrix[v, u] = cv_image[pix[0], pix[1]]
                    print("u=%d, v=%d, num=%d" %(u, v, matrix[v, u]))
                    # sys.stdout.write('Topic: %s: Depth at center(%d, %d): %d mm\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
                    # sys.stdout.flush()
            arr = np.asarray(matrix)
            np.savetxt("/home/aspa2/rockmelon/MMXXII/mask_rcnn/tools/record/matrix.csv", arr, delimiter=",")
        except CvBridgeError as e:
            print(e)
            return"""


if __name__ == '__main__':
    """Initialise the ROS node and run the main code above"""
    rospy.init_node("listener")
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    rospy.spin()