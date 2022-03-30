#!/usr/bin/env python
import os
from turtle import color
import cv2
import numpy as np
import pyrealsense2 as rs
from pyrealsense2 import pipeline, config


pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

profile = pipeline.start(config)

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

depth_sensor = profile.get_device().first_depth_sensor() 
# depth_scale = depth_sensor.get_option(rs.option.depth_units)
depth_scale = depth_sensor.get_depth_scale()
depth_pixel = [330, 330]
depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_scale)
color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)

print("Depth Intrinsics:", depth_intrin)
print("Color Intrinsics:", color_intrin)
print("Depth-to-Color Intrinsics:", depth_to_color_extrin)

print("Depth Sensor:", depth_sensor)
print("Depth Scale:", depth_scale)
print("Depth Pixel:", depth_pixel)
print("Depth Point:", depth_point)
print("Color Point:", color_point)
print("Color Pixel:", color_pixel)




'''# Start streaming
config = rs.config()
profile = pipeline.start(config)
# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth = depth_image[640,480].astype(float)
        distance = depth * depth_scale

        print ("Distance (m): ", distance)
except:
    pass'''