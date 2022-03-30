#!/usr/bin/env python
# import cv2
# import open3d as o3d
# import matplotlib.pyplot as plt
# from PIL import Image
# from resizeimage import resizeimage

# with open("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 1, "r+b") as f:
# 	with Image.open(f) as image:
# 		cover = resizeimage.resize_cover(image, [848,480])
# 		cover.save("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 9, image.format)

# color = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 9)
# depth = o3d.io.read_image("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 2)

# rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = True)

# plt.subplot(1, 2, 1)
# plt.title('Grayscale image')
# plt.imshow(rgbd.color)
# plt.subplot(1, 2, 2)
# plt.title('Depth image')
# plt.imshow(rgbd.depth)
# plt.show()

# '''#cam = o3d.camera.PinholeCameraIntrinsic(width= 848, height=480, cx=424,cy=240,fx=50,fy=50)
# cam = o3d.camera.PinholeCameraIntrinsic()
# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)'''
# camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, camera_intrinsic)

# '''downpcd = pcd.voxel_down_sample(voxel_size=0.05)
# o3d.visualization.draw_geometries([downpcd], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
# #o3d.visualization.draw_geometries([downpcd])
# '''
# # flip the orientation, so it looks upright, not upside-down
# pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

# voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=1)

# o3d.visualization.draw_geometries([pcd], point_show_normal=True, mesh_show_wireframe=True, mesh_show_back_face=True, zoom=0.5)





























































# # pseudocode for the rgb and depth image registration

# function [aligned] = ...
#                     depth_rgb_registration(depthData, rgbData,...
#                     fx_d, fy_d, cx_d, cy_d,...
#                     fx_rgb, fy_rgb, cx_rgb, cy_rgb,...
#                     extrinsics)

#     depthHeight = size(depthData, 1);
#     depthWidth = size(depthData, 2);
    
#     % Aligned will contain X, Y, Z, R, G, B values in its planes
#     aligned = zeros(depthHeight, depthWidth, 6);

#     for v = 1 : (depthHeight)
#         for u = 1 : (depthWidth)
#             % Apply depth intrinsics
#             z = single(depthData(v,u)) / depthScale;
#             x = single((u - cx_d) * z) / fx_d;
#             y = single((v - cy_d) * z) / fy_d;
            
#             % Apply the extrinsics
#             transformed = (extrinsics * [x;y;z;1])';
#             aligned(v,u,1) = transformed(1);
#             aligned(v,u,2) = transformed(2);
#             aligned(v,u,3) = transformed(3);
#         end
#     end

#     for v = 1 : (depthHeight)
#         for u = 1 : (depthWidth)
#             % Apply RGB intrinsics
#             x = (aligned(v,u,1) * fx_rgb / aligned(v,u,3)) + cx_rgb;
#             y = (aligned(v,u,2) * fy_rgb / aligned(v,u,3)) + cy_rgb;
            
#             % "x" and "y" are indices into the RGB frame, but they may contain
#             % invalid values (which correspond to the parts of the scene not visible
#             % to the RGB camera.
#             % Do we have a valid index?
#             if (x > rgbWidth || y > rgbHeight ||...
#                 x < 1 || y < 1 ||...
#                 isnan(x) || isnan(y))
#                 continue;
#             end
            
#             % Need some kind of interpolation. I just did it the lazy way
#             x = round(x);
#             y = round(y);

#             aligned(v,u,4) = single(rgbData(y, x, 1);
#             aligned(v,u,5) = single(rgbData(y, x, 2);
#             aligned(v,u,6) = single(rgbData(y, x, 3);
#         end
#     end    
# end





























































import os
import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from transforms3d import quaternions


def rgb_depth_registration(depth, color, extrinsic):
    depth_scale = 0.0010000000474974513

    '''fx_d = K_depth[0,0]
    fy_d = K_depth[1,1]
    cx_d = K_depth[0,2]
    cy_d = K_depth[1,2]

    fx_rgb = K_col[0,0]
    fy_rgb = K_col[1,1]
    cx_rgb = K_col[0,2]
    cy_rgb = K_col[1,2]'''

    fx_d = 386.02
    fy_d = 386.02
    cx_d = 320
    cy_d = 240

    fx_rgb = 610.421
    fy_rgb = 610.508
    cx_rgb = 320
    cy_rgb = 240

    height = depth.shape[0]
    width = depth.shape[1]

    print(height, width)

    aligned = np.zeros((height,width,6))

    for v in range(height):
        for u in range(width):
            # apply depth intrinsics
            z = depth[v,u].sum() * depth_scale
            x = ((u - cx_d) * z) / fx_d
            y = ((v - cy_d) * z) / fy_d
            
            # apply extrinsic
            transformed = np.dot(extrinsic, np.array([x,y,z,1])).T
            aligned[v,u,0] = transformed[0]
            aligned[v,u,1] = transformed[1]
            aligned[v,u,2] = transformed[2]
            
    for v in range(height):
        for u in range(width):
            # apply rgb intrinsic
            if aligned[v,u,2] != 0:
                x = (aligned[v,u,0] * fx_rgb / aligned[v,u,2]) + cx_rgb
                y = (aligned[v,u,1] * fy_rgb / aligned[v,u,2]) + cy_rgb
            else:
                x = 0
                y = 0
            
            # end the out of bound pixels
            if x > width-1 or y > height-1 or x < 0 or y < 0:
                pass
            # need some interpolation
            else:
                x = int(round(x))
                y = int(round(y))
            
                aligned[v,u,3] = color[y, x, 0]
                aligned[v,u,4] = color[y, x, 1]
                aligned[v,u,5] = color[y, x, 2]
    
    # retrive depth value from our aligned version
    depth_img = np.zeros((height,width,3))
    for i in range (height):
        for j in range (width):
            depth_img[i,j] = aligned[i,j][0:3]
    # print(depth_img)
    try:
        plt.imshow(depth_img.astype(int))
        plt.imsave("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/aligned_depth.png", depth_img)
    except:
        pass

    # retrive RGB value from our aligned version
    rgb_img = np.zeros((height,width,3))
    for i in range (height):
        for j in range (width):
            rgb_img[i,j] = aligned[i,j][3:6]
    # print(rgb_img)
    try:
        plt.imshow(rgb_img.astype(int))
        plt.imsave("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/aligned_rgb.png", rgb_img)
    except:
        pass

    return depth_img, rgb_img


if __name__ == '__main__':
    # the rotation matrix and translation vector can be found by 
    # executing the command 'rs-enumerate-devices -c' in the terminal
    R = [[0.999899, 0.0136223, -0.00404269], [-0.0136225, 0.999907, -0.0000273666], [0.00404195, 0.0000824354, 0.999992]]
    t = [-0.0148238586261868, -3.90679779229686e-05, -0.000269027281319723]

    # pose of camera
    extrinsic=np.zeros([4,4])
    for i in range(3):
        for j in range(3):
            extrinsic[i][j]=R[i][j]

    extrinsic[3][3]=1
    for m in range(3):
        extrinsic[m][3]=t[m]
    
    print(extrinsic)

    ROOT_DIR = "/home/marvin/catkin_ws/src/camera_scripts/finalised/row1"

    color = cv2.imread(os.path.join(ROOT_DIR, "rgb_images/rgb_image1.png"))
    depth = cv2.imread(os.path.join(ROOT_DIR, "depth_images/depth_image1.png"))

    print(type(depth))

    source_color = o3d.io.read_image(os.path.join(ROOT_DIR, "rgb_images/rgb_image1.png"))
    source_depth = o3d.io.read_image(os.path.join(ROOT_DIR, "depth_images/depth_image1.png"))

    print(type(source_depth))

    depth_img, rgb_img = rgb_depth_registration(depth, color, extrinsic)

    # plt.imshow(depth_img.astype(int))
    # plt.imsave("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/aligned_depth.png", depth_img)

    # plt.imshow(rgb_img.astype(int))
    # plt.imsave("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/aligned_rgb.png", rgb_img)

    plt.imshow(rgb_img.astype(int))
    cv2.imwrite("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/aligned_rgb.png", rgb_img)

    print(type(depth_img))

    depth_img = o3d.geometry.Image(np.asarray(depth_img).astype("uint8"))
    rgb_img = o3d.geometry.Image(np.asarray(rgb_img).astype("uint8"))

    print(np.shape(depth_img))
    print(np.shape(rgb_img))

    # rgbd_image=o3d.geometry.RGBDImage.create_from_color_and_depth(source_color, source_depth, 1000, 4, False)
    rgbd_image=o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, depth_img, depth_scale=0.0010000000474974513, convert_rgb_to_intensity=False)


    # camera intrinsic parameters
    cx = 317.879608154297
    cy = 244.293426513672
    # depth intrinsics
    # fx = 386.02
    # fy = 386.02
    # color intrinsics
    fx = 610.421203613281
    fy = 610.507629394531
    w = 640
    h = 480

    target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy), extrinsic)
    target_pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

    o3d.visualization.draw_geometries([target_pcd])
