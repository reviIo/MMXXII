#!/usr/bin/env python
import open3d as o3d
import matplotlib.pyplot as plt

print("Reading image dataset")
color_raw = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 1)
depth_raw = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 2)
print(type(color_raw))
print(type(depth_raw))
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw, convert_rgb_to_intensity=False)


'''plt.subplot(1, 2, 1)
plt.title('Grayscale Image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Depth Image')
plt.imshow(rgbd_image.depth)
plt.show()

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd], zoom=0.5)'''


