#!/usr/bin/env python
import sys
import message_filters
import rospy
import numpy as np
import open3d as o3d

from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

######### The RGB stream works very well, but the depth stream is a different story #########

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

def get_image_stream(rgb_msg, depth_msg):
    global vis
    global frame_num, fps_list

    try:
        rgb_img = CvBridge()
        rgb_img = rgb_img.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth_img = CvBridge()
        depth_img = depth_img.imgmsg_to_cv2(depth_msg, "8UC1")
    except CvBridgeError as e:
        print(e)

    start_time = datetime.now()
    #depth_img = np.expand_dims(depth_img, axis=-1)

    #rgb_img = np.array(rgb_img, dtype=np.uint8)
    #depth_img = np.array(depth_img, dtype=np.uint8)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb_img), o3d.geometry.Image(depth_img), convert_rgb_to_intensity = False
    )
    
    '''vis.update_geometry(rgbd_image)
    if not vis.poll_events():
        # Note that this should only terminate its own thread
        vis.close()
        print("The average fps is %.3f" % (sum(fps_list[:-1])/(len(fps_list)-1)))
        rospy.signal_shutdown("")
    vis.update_renderer()

    process_time = datetime.now() - start_time
    print("Frame Number %d" % frame_num)
    fps = 1/process_time.total_seconds()
    print("FPS = %d" % (fps))
    fps_list.append(fps)

    vis.clear_geometries()
    vis.add_geometry(rgbd_image)
    o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.35)'''

    vis.update_geometry(depth_img)
    if not vis.poll_events():
        # Note that this should only terminate its own thread
        vis.close()
        print("The average fps is %.3f" % (sum(fps_list[:-1])/(len(fps_list)-1)))
        rospy.signal_shutdown("")
    vis.update_renderer()

    process_time = datetime.now() - start_time
    print("Frame Number %d" % frame_num)
    fps = 1/process_time.total_seconds()
    print("FPS = %d" % (fps))
    fps_list.append(fps)

    vis.clear_geometries()
    vis.add_geometry(depth_img)
    o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.35)

    frame_num += 1

if __name__=="__main__":
    vis = o3d.visualization.Visualizer()
    vis.create_window("Open3D Viewer")

    frame_num = 1
    fps_list = []

    try:
        rospy.init_node('listener', anonymous=True)

        rgb_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_msg = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)

        ats = message_filters.ApproximateTimeSynchronizer([rgb_msg, depth_msg], 10, 0.1, allow_headerless=True)
        ats.registerCallback(get_image_stream)

        rospy.spin()
    except KeyboardInterrupt:
        sys.exit()