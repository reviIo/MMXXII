#!/usr/bin/env python
import rospy
import cv2
import ctypes
import struct
import threading
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from imutils import perspective
from datetime import datetime


def pc_callback(pointcloud):
	xyz = np.array([[0,0,0]])
	rgb = np.array([[0,0,0]])
	num = 0
    
	transform = pc2.read_points(pointcloud, skip_nans=True)
	data = list(transform)
	
	print(len(data))
	
	for x in data:
		num += 1
		print(num)
		
		test = x[3]
        # cast float32 to int so that bitwise operations are possible
		s = struct.pack('>f' ,test)
		i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
		pack = ctypes.c_uint32(i).value
		r = (pack & 0x00FF0000) >> 16
		g = (pack & 0x0000FF00) >> 8
		b = (pack & 0x000000FF)
		# prints r,g,b values in the 0-255 range
		# x,y,z can be retrieved from the x[0],x[1],x[2]
		xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
		rgb = np.append(rgb,[[r,g,b]], axis = 0)
		
    
	print("This line is executed")
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(xyz)
	pcd.colors = o3d.utility.Vector3dVector(rgb.astype(np.float) / 255.0)
	o3d.io.write_point_cloud("/home/marvin/catkin_ws/src/camera_scripts/images/pointcloud.ply",pcd)



def listener():
	global num
    
	input("Press Enter to take an image")
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('listener', anonymous=True)
    
	rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback=pc_callback)
	
def stream_pointcloud():
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream different resolutions of color and depth streams
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

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
        o3d.geometry_added = False
        #vis = Visualizer()
        vis = o3d.visualization.Visualizer()
        #vis.create_window('Aligned Column', width=640, height=480)
        vis.create_window("Test")
        pcd = o3d.geometry.PointCloud()
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
            pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
            )
            
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            

            if not aligned_depth_frame or not color_frame:
                continue
            
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            img_color = o3d.geometry.Image(color_image)
            img_depth = o3d.geometry.Image(depth_image)
            
            #Create RGBD image
            #rgbd_image = open3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth, depth_scale*1000, depth_trunc=2000, convert_rgb_to_intensity=False)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth,convert_rgb_to_intensity=False)

            ##USE DEPTH CAMERA INTRINSICS
            #Create PC from RGBD
            #temp = open3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic_od3)
            
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
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



	
'''def view_pointcloud():
	# Reading and visualizing the point cloud
	cloud = o3d.io.read_point_cloud("/home/marvin/catkin_ws/src/camera_scripts/images/pointcloud.ply")
	o3d.visualization.draw_geometries([cloud])

def view_example():
	# Reading and visualizing the point cloud
	cloud = o3d.io.read_point_cloud("/home/marvin/catkin_ws/src/camera_scripts/images/car-wheel-cap-ply/CarWheelCap.ply")
	o3d.visualization.draw_geometries([cloud])'''
	

if __name__ == '__main__':
    try:
        thread1 = threading.Thread(target=listener)
        thread2 = threading.Thread(target=stream_pointcloud)
        
        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()

        #view_pointcloud()
    except KeyboardInterrupt():
        exit()