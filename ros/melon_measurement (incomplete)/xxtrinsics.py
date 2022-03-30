#!/usr/bin/env python


# # pseudocode for the rgb and depth image registration
# # https://www.codefull.net/2016/03/align-depth-and-color-frames-depth-and-rgb-registration/

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
    # Data obtained from the Intel RealSense D435i camera properties
    depth_scale = 0.0010000000474974513

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

    aligned = np.zeros((height,width,6))

    for v in range (height):
        for u in range (width):
            # Apply depth intrinsics
            z = depth[v,u].sum() * depth_scale
            x = ((u - cx_d) * z) / fx_d
            y = ((v - cy_d) * z) / fy_d
            
            # Apply extrinsic
            transformed = np.dot(extrinsic, np.array([x,y,z,1])).T
            aligned[v,u,0] = transformed[0]
            aligned[v,u,1] = transformed[1]
            aligned[v,u,2] = transformed[2]
            
    for v in range (height):
        for u in range (width):
            # Apply rgb intrinsic
            if aligned[v,u,2] != 0:
                x = (aligned[v,u,0] * fx_rgb / aligned[v,u,2]) + cx_rgb
                y = (aligned[v,u,1] * fy_rgb / aligned[v,u,2]) + cy_rgb
            else:
                x = cx_rgb
                y = cx_rgb
            
            # End the out of bound pixels
            if x > width-1 or y > height-1 or x < 0 or y < 0:
                pass
            
            else:
                x = int(round(x))
                y = int(round(y))
            
                aligned[v,u,3] = color[y, x, 0]
                aligned[v,u,4] = color[y, x, 1]
                aligned[v,u,5] = color[y, x, 2]
    
    # Retrive depth value from our aligned version
    depth_img = np.zeros((height,width,3))
    for i in range (height):
        for j in range (width):
            depth_img[i,j] = aligned[i,j][0:3]
    
    try:
        plt.imshow(depth_img.astype(int))
        plt.imsave("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/aligned_depth.png", depth_img)
    except:
        pass

    # Retrive RGB value from our aligned version
    rgb_img = np.zeros((height,width,3))
    for i in range (height):
        for j in range (width):
            rgb_img[i,j] = aligned[i,j][3:6]

    try:
        plt.imshow(rgb_img.astype(int))
        plt.imsave("/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/aligned_rgb.png", rgb_img)
    except:
        pass

    return depth_img, rgb_img


if __name__ == '__main__':
    # Need to modify these numbers
    R=quaternions.quat2mat([0.7008417, -0.7132461, -0.0012983065, 0.0099677965])
    t=np.array([-0.01873879, -0.0, -0.0])

    # Pose of camera
    extrinsic=np.zeros([4,4])
    for i in range(3):
        for j in range(3):
            extrinsic[i][j]=R[i][j]

    extrinsic[3][3]=1
    for m in range(3):
        extrinsic[m][3]=t[m]
    
    print(extrinsic)

    ROOT_DIR = "/home/marvin/catkin_ws/src/camera_scripts/finalised/row1"

    # Read the images to be remapped and transform them
    color = cv2.imread(os.path.join(ROOT_DIR, "rgb_images/rgb_image1.png"))
    depth = cv2.imread(os.path.join(ROOT_DIR, "depth_images/depth_image1.png"))
    depth_img, rgb_img = rgb_depth_registration(depth, color, extrinsic)
