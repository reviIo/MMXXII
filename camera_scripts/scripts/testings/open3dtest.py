#!/usr/bin/env python
import pyrealsense2 as rs
import numpy as np
import cv2
from open3d import *

width = 848
height = 480

pipeline = rs.pipeline()
pc=rs.pointcloud()
cfg=pipeline.start()
profile = cfg.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
intrinsic = profile.as_video_stream_profile().get_intrinsics()
ph_intrinsic=camera.PinholeCameraIntrinsic()
ph_intrinsic.set_intrinsics(intrinsic.width, intrinsic.height, intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy)



frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame() 
align_to = rs.stream.depth
align = rs.align(align_to)
aligned_frames = align.process(frames)
# depth_frame_open3d = geometry.Image(np.array(depth_frame.get_data()))
# color_frame_open3d = geometry.Image(np.array([color_frame.get_data()[:848,:],color_frame.get_data()[:,:480]]))
# color_frame_open3d = geometry.Image(np.array(color_frame.get_data()))
depth_frame_open3d = geometry.Image(np.array(depth_frame.get_data()))
color_frame_open3d = geometry.Image(cv2.resize(np.array(color_frame.get_data()), (width, height), interpolation=cv2.INTER_AREA))
rgbd_image = geometry.RGBDImage.create_from_color_and_depth(color_frame_open3d, 			depth_frame_open3d)
pcd = geometry.PointCloud.create_from_rgbd_image(rgbd_image, ph_intrinsic)
visualization.draw_geometries([pcd])
