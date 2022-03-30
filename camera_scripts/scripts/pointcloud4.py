#!/usr/bin/env python
import cv2
import numpy as np
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

#print(np.shape(depth))

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)

'''plt.subplot(1, 2, 1)
plt.title('Grayscale image')
plt.imshow(rgbd.color)
plt.subplot(1, 2, 2)
plt.title('Depth image')
plt.imshow(rgbd.depth)
plt.show()'''

'''#cam = o3d.camera.PinholeCameraIntrinsic(width= 848, height=480, cx=424,cy=240,fx=50,fy=50)
cam = o3d.camera.PinholeCameraIntrinsic()
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)'''
camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, camera_intrinsic)

'''downpcd = pcd.voxel_down_sample(voxel_size=0.05)
o3d.visualization.draw_geometries([downpcd], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
#o3d.visualization.draw_geometries([downpcd])
'''
# flip the orientation, so it looks upright, not upside-down
pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=1)

o3d.visualization.draw_geometries([pcd], point_show_normal=True, mesh_show_wireframe=True, mesh_show_back_face=True, zoom=0.5)
