#!/usr/bin/env python
import os
import cv2
import json
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from datetime import datetime


print(os.listdir("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles"))

### Initialising RealSense Camera ###

o3d.t.io.RealSenseSensor.list_devices()



### Capturing Video Stream using RealSense with .json File as Configuration ###

## with open("/home/marvin/catkin_ws/src/camera_scripts/scripts/rsconfig.json") as cf:
##    rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf))

# rs_cfg = o3d.t.io.RealSenseSensorConfig(o3d.cpu.pybind.t.io.RealSenseSensorConfig())

rs = o3d.t.io.RealSenseSensor()
rs.init_sensor(o3d.t.io.RealSenseSensorConfig(), 0, "/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/testing4.bag")
rs.start_capture(True)  # true: start recording with capture


geometry_added = False
vis = o3d.visualization.Visualizer()
#vis.create_window('Aligned Column', width=640, height=480)
vis.create_window("RealSense")
im_rgbd = o3d.geometry.PointCloud()

try:
    while True:
        im_rgbd = rs.capture_frame(True, True)  # wait for frames and align them
        # process im_rgbd.depth and im_rgbd.color

        print(type(im_rgbd))

        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
        )

        temp = o3d.geometry.PointCloud.create_from_rgbd_image(im_rgbd, camera_intrinsic)
        
        vis.add_geometry(im_rgbd)
        im_rgbd.clear()

        vis.update_geometry(temp)
        vis.poll_events()    
        vis.update_renderer()

        key = cv2.waitKey(1)   

        # if 'esc' button pressed, escape loop and exit program
        if key == 27:
            cv2.destroyAllWindows()
            break

except KeyboardInterrupt:
    rs.stop_capture()
    vis.destroy_window()


'''

### Viewing Bag Files ###


###bag_reader = o3d.t.io.RSBagReader()
###bag_reader.open("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/testing2.bag")
###while not bag_reader.is_eof():
    ###im_rgbd = bag_reader.next_frame()
    ### # process im_rgbd.depth and im_rgbd.color

###bag_reader.close()


bag_reader = o3d.t.io.RSBagReader()
bag_reader.open("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/testing3.bag")
im_rgbd = bag_reader.next_frame()

while not bag_reader.is_eof():
    im_rgbd = bag_reader.next_frame()
    cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Realsense', im_rgbd)
    key = cv2.waitKey(1)   
    
    # if 'esc' button pressed, escape loop and exit program
    if key == 27:
        cv2.destroyAllWindows()
        break
bag_reader.close()

'''