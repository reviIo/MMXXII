#!/usr/bin/env python
import os
import cv2
import rosbag
import argparse
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("rgb_topic", help="RGB image topic.")
    parser.add_argument("depth_topic", help="Depth image topic.")

    args = parser.parse_args()

    print("Extract a set of rgb and depth images from %s on topic %s into %s" % (args.bag_file,
                                                          (args.rgb_topic, args.depth_topic), args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    rgb_bridge = CvBridge()
    depth_bridge = CvBridge()
    count = 0
    '''for topic, depth_msg, t in bag.read_messages(topics=args.depth_topic):
        depth_img = depth_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        depth_dir = os.path.join(args.output_dir, "/depth_extracted/depth_extracted%s.raw" % count)
        print(depth_dir)

        # cv2.imwrite(os.path.join(args.output_dir, "/rgb_extracted/rgb_extracted%s.png" % count), rgb_img)
        # np.save(os.path.join(args.output_dir, "/depth_extracted/depth_extracted%s.raw" % count), depth_img)
        depth_img.astype('int16').tofile("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/depth_extracted/depth_extracted%s.raw" % count)

        print("Wrote image set %i\r" % count)

        count += 1'''
    
    for topic, rgb_msg, t in bag.read_messages(topics=args.rgb_topic):
        rgb_img = rgb_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        
        rgb_dir = os.path.join(args.output_dir, "/rgb_extracted/rgb_extracted%s.png" % count)
        print(rgb_dir)

        cv2.imwrite(os.path.join("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/rgb_extracted/rgb_extracted%s.png" % count), rgb_img)
        #rgb_img.astype('int16').tofile("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/rgb_extracted/rgb_extracted%s.raw" % count)

        print("Wrote image set %i\r" % count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()


# #!/usr/bin/env python
# import cv2
# import pyrealsense2 as rs

# # img = cv2.imread("/home/marvin/mlfolder/MMXXII/row1/depth_images/depth_image1.png")

# # dist = img.get_distance(320, 240)
# # print(dist)

# # '''depth_info = rs.depth_frame()
# # distance = depth_info.get_distance(x=320, y=240)'''

# ## License: Apache 2.0. See LICENSE file in root directory.
# ## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

# #####################################################
# ## librealsense tutorial #1 - Accessing depth data ##
# #####################################################

# # First import the library
# import pyrealsense2 as rs

# try:
#     # Create a context object. This object owns the handles to all connected realsense devices
#     pipeline = rs.pipeline()

#     # Configure streams
#     config = rs.config()
#     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

#     # Start streaming
#     pipeline.start(config)

#     while True:
#         # This call waits until a new coherent set of frames is available on a device
#         # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
#         frames = pipeline.wait_for_frames()
#         depth = frames.get_depth_frame()
#         print(depth)
#         if not depth: continue

#         # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
#         coverage = [0]*64
#         # for y in range(480):
#         #     for x in range(640):
#         #         dist = depth.get_distance(x, y)
#         #         print("{:1f}".format(dist))
#         '''dist = depth.get_distance(320, 240)
#         print("{:1f}".format(dist))'''

#         img = cv2.imread("/home/marvin/mlfolder/MMXXII/row1/depth_images/depth_image190.png")
#         print(img[240][320])

#             #     if 0 < dist and dist < 1:
#             #         coverage[x//10] += 1
            
#             # if y%20 is 19:
#             #     line = ""
#             #     for c in coverage:
#             #         line += " .:nhBXWW"[c//25]
#             #     coverage = [0]*64
#             #     print(line)
#     exit(0)
# #except rs.error as e:
# #    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
# #    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
# #    print("    %s\n", e.what())
# #    exit(1)
# except Exception as e:
#     print(e)
#     pass
