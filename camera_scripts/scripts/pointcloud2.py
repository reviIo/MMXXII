#!/usr/bin/env python
import cv2
import open3d as o3d 
import numpy as np
from PIL import Image
from resizeimage import resizeimage

with open("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 1, "r+b") as f:
	with Image.open(f) as image:
		cover = resizeimage.resize_cover(image, [848,480])
		cover.save("/home/marvin/catkin_ws/src/camera_scripts/images/image111%s.png" % 1, image.format)
		
color = cv2.imread("/home/marvin/catkin_ws/src/camera_scripts/images/image111%s.png" % 1)
depth = cv2.imread("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 2)

print(type(color))
print(type(depth))

color = o3d.geometry.Image((color * 255).astype(np.uint8))
depth = o3d.geometry.Image((depth[-1]*1000).astype(np.uint16))

print(type(color))
print(type(depth))

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)
cam = o3d.camera.PinholeCameraIntrinsic()
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)

# flip the orientation, so it looks upright, not upside-down
pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)
# pcd.colors = o3d.utility.Vector3dVector(colors/65535)
# pcd.normals = o3d.utility.Vector3dVector(normals)

o3d.visualization.draw_geometries([pcd])

'''o3d.visualization.draw_geometries([pcd],zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])    # visualize the point cloud'''
# o3d.visualizer.show()
