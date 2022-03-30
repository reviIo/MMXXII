#!/usr/bin/env python
import os
from PIL import Image

num = 1

directory = r"/home/aspa2/rockmelon/MMXXII/row1/rgb_images/"

for filename in os.listdir(directory):
    print(filename)
    if filename.endswith(".png"): 
        im = Image.open(os.path.join(directory, filename))
        rgb_im = im.convert('RGB')
        rgb_im.save(r"/home/aspa2/rockmelon/MMXXII/row1/png2jpg/rgb_image%s.jpg" %(num))
        num+=1
    else:
        continue
