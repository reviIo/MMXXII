#!/usr/bin/env python
import os
import rosbag
import numpy as np
import pyrealsense2 as rs

# bagfile = rosbag.Bag("/home/marvin/catkin_ws/src/camera_scripts/finalised/recording/row1_2022-01-12-12-54-49.bag")
# print(bagfile)
# msgs = [msg for topic, msg, t in bagfile.read_messages(topics=['/camera/depth/image_rect_raw'])]
# print(msgs)



# # Intel Realsense Live Feed Get Distance

# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# pipeline.start(config)
# count = 0
# try:
#     while True:
#         frames = pipeline.wait_for_frames()
#         depth = frames.get_depth_frame()

#         if not depth: continue

#         count += 1
#         x1, y1 = 80, 30
#         midx, midy = 640, 360

#         #Pixel 1
#         dist = depth.get_distance(x1, y1)
#         dist = dist*1000
#         print("Pixel 1 Distance in mm:", dist)
        
#         #Mid Pixel
#         dist = depth.get_distance(midx, midy)
#         dist = dist*1000
#         # print("mid Pixel Distance in mm:", dist)

#     pipeline.stop()
#     exit(0)

# except Exception as e:
#     print(e)
#     pass



# Frames Alignment

    # import pyrealsense2 as rs

    # pipeline = None
    # colorizer = rs.colorizer()
    # align = rs.align(rs.stream.depth)

    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30) 
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    # profile = pipeline.start(config)

    # frameset = pipeline.wait_for_frames()
    # frameset = align.process(frameset)
    # aligned_color_frame = frameset.get_color_frame()
    # color_frame = np.asanyarray(aligned_color_frame.get_data())
    # depth = np.asanyarray(frameset.get_depth_frame().get_data())

import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import os


def extract_from_bag(bag_fname, color_fname, depth_fname):

    config = rs.config()
    pipeline = rs.pipeline()

    # make it so the stream does not continue looping
    config.enable_stream(rs.stream.color)
    config.enable_stream(rs.stream.depth)
    rs.config.enable_device_from_file(config, bag_fname, repeat_playback=False)
    profile = pipeline.start(config)
    # this makes it so no frames are dropped while writing video
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    colorizer = rs.colorizer()

    align_to = rs.stream.color
    align = rs.align(align_to)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    depth_matrices = []

    i = 0
    while True:

        # when stream is finished, RuntimeError is raised, hence this
        # exception block to capture this
        try:
            # frames = pipeline.wait_for_frames()
            frames = pipeline.wait_for_frames(timeout_ms=100)
            if frames.size() <2:
                # Inputs are not ready yet
                continue
        except (RuntimeError):
            print('frame count', i-1)
            pipeline.stop()
            break

        # align the deph to color frame
        aligned_frames = align.process(frames)

        # get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        scaled_depth_image = depth_image * depth_scale
        color_image = np.asanyarray(color_frame.get_data())

        # convert color image to BGR for OpenCV
        r, g, b = cv2.split(color_image)
        color_image = cv2.merge((b, g, r))

        depth_colormap = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('Aligned Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Aligned Example', images)

        fname = "frame{:06d}".format(i) + ".png"
        cv2.imwrite(color_fname + fname, color_image)

        depth_matrices.append(scaled_depth_image)

        # color_out.write(color_image)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

        i += 1

    # release everything now that job finished
    np.save(depth_fname, np.array(depth_matrices))
    print("Size of depth matrices:", len(depth_matrices))
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-i", "--input", type=str, help=".bag file to read")
    # parser.add_argument("-c", "--rgbfilename", type=str, help=".mp4 file to save RGB stream")
    # parser.add_argument("-d", "--depthfilename", type=str, help=".npy file to save depth stream")
    # args = parser.parse_args()

    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=str, help=".bag file to read")
    parser.add_argument("rgbfilename", type=str, help=".mp4 file to save RGB stream")
    parser.add_argument("depthfilename", type=str, help=".npy file to save depth stream")
    args = parser.parse_args()

    extract_from_bag(bag_fname=args.input, color_fname=args.rgbfilename, depth_fname=args.depthfilename)





# # Alignment and Read Stream

# def main():
#     # Setup:
#     # create pipeline
#     pipe = rs.pipeline()
#     # create config object
#     cfg = rs.config()
#     # tell config that we will use recorded device from file
#     # to be used by the pipeline through playback
#     cfg.enable_device_from_file("../data/distance_test.bag")
#     #cfg.enable_all_streams()
#     align_to = rs.stream.infrared
#     align = rs.align(align_to)
#     # start streaming from file
#     profile = pipe.start(cfg)
#     # setup colorizer for depthmap
#     colorizer = rs.colorizer()
#     # setup alignment
#     align_to = rs.stream.infrared
#     align = rs.align(align_to)

#     # setup playback
#     playback = profile.get_device().as_playback()
#     playback.set_real_time(False)
#     # get the duration of the video
#     t = playback.get_duration()
#     t.seconds
#     # compute the number of frames (30fps setting)
#     frame_counts = t.seconds * 30

#     # extract and save all depth frames
#     result = []
#     spec = 0
#     ir_frames = []
#     for i in range(frame_counts):
#         #print(i)
#         frame = pipe.wait_for_frames()
#         aligned = align.process(frame)
#         ir = aligned.get_infrared_frame()
#         ir_frames.append(ir)
#         depth = aligned.get_depth_frame()
#         #result.append(np.asanyarray(colorizer.colorize(depth).get_data()))
#         #tmp = np.asanyarray(colorizer.colorize(depth).get_data())
#         result.append(depth)
#         #io.imsave("../outputs/distance_test/" + str(i) + ".jpg", tmp)
#         #result.append(tmp)
#         #if i == 16:
#             #spec = np.asanyarray(colorizer.colorize(depth).get_data())
#             #color = frame.get_color_frame()
#     playback.pause()
#     pipe.stop()

#     # Fit the following suggestions/corrections into the code
#     configs = []
#     for f,i in zip(files, range(len(files)):
#     c = rs.config()
#     c.enable_device_from_file(filepath, repeat_playback=False)
#     configs.append(c)
#     frames = load_single_frame_from_file(configs[i], f)

#     with

#     def load_single_frame_from_file(rs_config)":
#     pipeline = rs.pipeline()
#     profile = pipeline.start(rs_config)
#     frameset = pipeline.wait_for_frames()
#     streams = profile.get_streams()

#     return frame