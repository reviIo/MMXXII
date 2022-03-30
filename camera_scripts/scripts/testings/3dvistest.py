#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import sin, cos
  
fig = plt.figure(figsize = (8,8))
ax = fig.add_subplot(111, projection = '3d')
  
#creating Datasheet
y = np.linspace(-1, 1, 200)
x = np.linspace(-1, 1, 200)
x,y = np.meshgrid(x, y)

color = cv2.imread("/home/marvin/catkin_ws/src/camera_scripts/images/image111%s.png" % 1)
depth = cv2.imread("/home/marvin/catkin_ws/src/camera_scripts/images/image%s.png" % 2)
  
#set z values
z = x + y 
  
# rotate the samples by changing the value of 'a'
a = 50 
  
t = np.transpose(np.array([x, y, z]), ( 1, 2, 0))
  
m = [[cos(a), 0, sin(a)],[0, 1, 0],
     [-sin(a), 0, cos(a)]]
  
X,Y,Z = np.transpose(np.dot(t, m), (2, 0, 1))
  
#label axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
  
#plot figure
ax.plot_surface(X,Y,Z, alpha = 0.5,
                color = 'red')
  
plt.show()
