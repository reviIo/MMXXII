#!/usr/bin/env python
import os
import cv2
import json
import threading
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

from ROS2Open3D import convert_rgbUint32_to_tuple, convert_rgbFloat_to_tuple, convertCloudFromRosToOpen3d
from datetime import datetime
from sensor_msgs.msg import PointCloud2 as pc2


print(os.listdir("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles"))

### Initialising RealSense Camera ###

o3d.t.io.RealSenseSensor.list_devices()

'''

### Capturing Video Stream using RealSense with .json File as Configuration ###

## with open("/home/marvin/catkin_ws/src/camera_scripts/scripts/rsconfig.json") as cf:
##    rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf))

# rs_cfg = o3d.t.io.RealSenseSensorConfig(o3d.cpu.pybind.t.io.RealSenseSensorConfig())

rs = o3d.t.io.RealSenseSensor()
rs.init_sensor(o3d.t.io.RealSenseSensorConfig(), 0, "/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/testing3.bag")
rs.start_capture(True)  # true: start recording with capture

while True:
    im_rgbd = rs.capture_frame(True, True)  # wait for frames and align them
    # process im_rgbd.depth and im_rgbd.color
    color_np = im_rgbd.color.cpu().as_tensor().numpy()
    cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Realsense', color_np)
    key = cv2.waitKey(1)   
    
    # if 'esc' button pressed, escape loop and exit program
    if key == 27:
        cv2.destroyAllWindows()
        break
rs.stop_capture()


'''

### Viewing Bag Files ###


###bag_reader = o3d.t.io.RSBagReader()
###bag_reader.open("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/testing2.bag")
###while not bag_reader.is_eof():
    ###im_rgbd = bag_reader.next_frame()
    ### # process im_rgbd.depth and im_rgbd.color

###bag_reader.close()

'''def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud'''

bag_reader = o3d.t.io.RSBagReader()
bag_reader.open("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/20211130_RV.bag")
#bag_reader.open("/home/marvin/catkin_ws/src/camera_scripts/scripts/bagfiles/2021-11-29-09-25-19.bag")
im_rgbd = bag_reader.next_frame()

vis = o3d.visualization.Visualizer()
vis.create_window("Open3D Viewer")

frame_num = 1
fps_list = []

while not bag_reader.is_eof():
    start_time = datetime.now()
    im_rgbd = bag_reader.next_frame()
    
    rgb = np.array(im_rgbd.color)
    depth = np.array(im_rgbd.depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb), o3d.geometry.Image(depth), convert_rgb_to_intensity = False
    )
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    pointcloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
    
    pointcloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(im_rgbd.color), o3d.geometry.Image(im_rgbd.depth))
    '''vis.update_geometry(pointcloud)
    if not vis.poll_events():
        break
    vis.update_renderer()'''

    vis.update_geometry(rgbd_image)
    if not vis.poll_events():
        break
    vis.update_renderer()

    process_time = datetime.now() - start_time
    print("Frame Number %d" % frame_num)
    fps = 1/process_time.total_seconds()
    print("FPS = %d" % (fps))
    fps_list.append(fps)

    '''vis.clear_geometries()
    vis.add_geometry(pointcloud)
    o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.35)'''

    vis.clear_geometries()
    vis.add_geometry(rgbd_image)
    o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.35)

    frame_num += 1

    # cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('Realsense', im_rgbd)
    # key = cv2.waitKey(1)   
    
    # # if 'esc' button pressed, escape loop and exit program
    # if key == 27:
    #     cv2.destroyAllWindows()
    #     break
bag_reader.close()

print("The average fps is %.3f" % (sum(fps_list[:-1])/(len(fps_list)-1)))