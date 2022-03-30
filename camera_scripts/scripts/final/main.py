#!/usr/bin/env python
import cv2
import rospy
import threading
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from PIL import Image
from datetime import datetime
from sensor_msgs.msg import Image
from resizeimage import resizeimage
from cv_bridge import CvBridge, CvBridgeError

###############################################################################################################

# Capturing a RGB and depth image and viewing it

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

    plt.subplot(1, 2, 1)
    plt.title('RGB image')
    plt.imshow(img1.color)
    plt.subplot(1, 2, 2)
    plt.title('Depth image')
    plt.imshow(img2.depth)
    plt.show()

###############################################################################################################

# Viewing the 3D pointcloud from RGB and depth images saved previously

with open("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 1, "r+b") as f:
	with Image.open(f) as image:
		cover = resizeimage.resize_cover(image, [848,480])
		cover.save("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 9, image.format)

color = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 9)
depth = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 2)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = True)

'''#cam = o3d.camera.PinholeCameraIntrinsic(width= 848, height=480, cx=424,cy=240,fx=50,fy=50)
cam = o3d.camera.PinholeCameraIntrinsic()
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)'''
camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, camera_intrinsic)


'''vol = o3d.visualization.read_selection_polygon_volume(
    "../../test_data/Crop/cropped.json")
roi = vol.crop_point_cloud(pcd)
o3d.visualization.draw_geometries([roi],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])'''


'''downpcd = pcd.voxel_down_sample(voxel_size=0.05)
#o3d.visualization.draw_geometries([downpcd], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
o3d.visualization.draw_geometries([downpcd])'''

# flip the orientation, so it looks upright, not upside-down
pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

#voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.5)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
#o3d.visualization.ViewControl.set_front(vis.get_view_control(), [0.9288, -0.2951, -0.2242])
#o3d.visualization.ViewControl.set_lookat(vis.get_view_control(), [1.6784, 2.0612, 1.4451])
#o3d.visualization.ViewControl.set_up(vis.get_view_control(), [-0.3402, -0.9189, -0.1996])
o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.45)
vis.run()

###############################################################################################################

# Recording video stream of the RGB and depth image into a bag file



###############################################################################################################

# Visualising the captured image



###############################################################################################################

# Visualising the recorded video



###############################################################################################################

# Bag file reader

bag_reader = o3d.t.io.RSBagReader()
bag_reader.open("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/20211130_162106.bag")
#bag_reader.open("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/2021-11-29-09-25-19.bag")
im_rgbd = bag_reader.next_frame()

vis = o3d.visualization.Visualizer()
vis.create_window("Open3D Viewer")

frame_num = 1
fps_list = []

while not bag_reader.is_eof():
    start_time = datetime.now()
    im_rgbd = bag_reader.next_frame()
    
    rgb = np.array(im_rgbd.color)
    depth = np.array(im_rgbd.depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb), o3d.geometry.Image(depth), convert_rgb_to_intensity = False
    )
    
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(im_rgbd.color), o3d.geometry.Image(im_rgbd.depth))
    vis.update_geometry(rgbd_image)
    if not vis.poll_events():
        break
    vis.update_renderer()

    process_time = datetime.now() - start_time
    print("Frame Number %d" % frame_num)
    fps = 1/process_time.total_seconds()
    print("FPS = %d" % (fps))
    fps_list.append(fps)

    vis.clear_geometries()
    vis.add_geometry(rgbd_image)

    frame_num += 1

bag_reader.close()

print("The average fps is %.3f" % (sum(fps_list[:-1])/(len(fps_list)-1)))

###############################################################################################################

# The main function

if __name__ == '__main__':
    try:
        thread1 = threading.Thread()
        thread2 = threading.Thread()
        thread3 = threading.Thread()
        thread1.start()
        thread2.start()
        thread3.start()
        thread1.join()
        thread2.join()
        thread3.join()
        while True:
            listener()
    except KeyboardInterrupt:
    	exit()