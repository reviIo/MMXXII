#!/usr/bin/env python
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

num = 0
td_num = 0
td_lst = []

rgb_img = cv2.imread("/home/aspa2/rockmelon/MMXXII/row1/rgb_images/rgb_image1.png", cv2.IMREAD_COLOR)
print(rgb_img)

print(len(rgb_img[0])) 

depth_img = cv2.imread("/home/aspa2/rockmelon/MMXXII/row1/depth_images/depth_image1.png", cv2.IMREAD_GRAYSCALE)
print(depth_img)

print(len(depth_img[0]))

# for i in depth_img:
#     for j in i:
#         if j > 0: 
#             print(j)
#             num += 1
#         if j > 9: 
#             td_lst.append(j)
#             td_num += 1

print(num)
print(td_num)
print(td_lst)
np.set_printoptions(threshold=np.inf)
print(depth_img)

plt.figure("Visualisation", figsize=(12, 8))
plt.subplot(1,2,1)
plt.title('RGB Image')
plt.imshow(rgb_img)
plt.subplot(1,2,2)
plt.title('Depth Image')
plt.imshow(depth_img)
plt.show(block=True)