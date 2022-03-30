#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
from io import StringIO
import PIL

scene_infile = open('/home/marvin/catkin_ws/src/camera_scripts/finalised/tools/depth_extracted/depth_extracted55.raw','rb')
scene_image_array = np.fromfile(scene_infile,dtype=np.uint8,count=640*480)
scene_image = PIL.Image.frombuffer("I",[640,480],
                                 scene_image_array.astype('I'),
                                 'raw','I',0,1)
plt.imshow(scene_image)
plt.show()