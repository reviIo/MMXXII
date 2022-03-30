#!/usr/bin/env python
import cv2
import open3d as o3d
import matplotlib.pyplot as plt
from PIL import Image
from resizeimage import resizeimage

with open("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 1, "r+b") as f:
	with Image.open(f) as image:
		cover = resizeimage.resize_cover(image, [848,480])
		cover.save("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 9, image.format)

color = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 9)
depth = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 2)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = True)
cam = o3d.camera.PinholeCameraIntrinsic(width= 848, height=480, cx=424,cy=240,fx=50,fy=50)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)

# flip the orientation, so it looks upright, not upside-down
pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

'''vis = o3d.visualization.Visualizer()
points = vis.capture_screen_float_buffer(do_render=False)
plt.imshow(points)
plt.axis('off')
plt.show()'''

pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)
# pcd.colors = o3d.utility.Vector3dVector(colors/65535)
# pcd.normals = o3d.utility.Vector3dVector(normals)

o3d.visualization.draw_geometries([pcd])

'''o3d.visualization.draw_geometries([pcd],zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])    # visualize the point cloud
#o3d.visualizer.show()'''
