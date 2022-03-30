#!/usr/bin/env python
import cv2
import numpy as np
import pyrealsense2 as rs
from imutils import perspective
from datetime import datetime
from open3d import *

pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream different resolutions of color and depth streams
config = rs.config()

config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, 30)

#Start streaming
profile = pipeline.start(config)

#Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()

#Turn ON Emitter
depth_sensor.set_option(rs.option.emitter_always_on, 1)

#Align
align_to = rs.stream.color
align = rs.align(align_to)

#Streaming loop
try:
    geometry_added = False
    #vis = Visualizer()
    vis = open3d.visualization.Visualizer()
    #vis.create_window('Aligned Column', width=640, height=480)
    vis.create_window("Test")
    pcd = open3d.geometry.PointCloud()
    while True:
        # Initialisation
        start_time = datetime.now()
        vis.add_geometry(pcd)
        pcd.clear()
        
        # Get frameset of color and depth

        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get Depth Intrinsics 
        # Return pinhole camera intrinsics for Open3d
        intrinsics = aligned_frames.profile.as_video_stream_profile().intrinsics
        pinhole_camera_intrinsic = open3d.camera.PinholeCameraIntrinsic(
            intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
        )
        
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        

        if not aligned_depth_frame or not color_frame:
            continue
        
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        img_color = open3d.geometry.Image(color_image)
        img_depth = open3d.geometry.Image(depth_image)
        
        #Create RGBD image
        #rgbd_image = open3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth, depth_scale*1000, depth_trunc=2000, convert_rgb_to_intensity=False)
        rgbd_image = open3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth,convert_rgb_to_intensity=False)

        ##USE DEPTH CAMERA INTRINSICS
        #Create PC from RGBD
        #temp = open3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic_od3)
        
        temp = open3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, pinhole_camera_intrinsic)
        
        
        #Flip it, otherwise the pointcloud will be upside down
        temp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        pcd.points=temp.points
        pcd.colors=temp.colors

            
        vis.update_geometry(pcd)
        vis.poll_events()    
        vis.update_renderer()
        
        #Calculate process time
        process_time = datetime.now() - start_time
        print("FPS = %d" %(1/process_time.total_seconds()))
        
        #Press esc or 'q' to close the image window
        if cv2.waitKey(0) & 0xFF == ord('q') or cv2.waitKey == 27:
            cv2.destroyAllWindows()
            break
except KeyboardInterrupt:
    pipeline.stop()
    
vis.destroy_window()
