#!/usr/bin/env python
import os
from PIL import Image

# Initialise the counter variable
num = 1

# Set the path to the directory with all the png 
# images that needs to be converted to jpg
directory = r"/home/aspa2/rockmelon/MMXXII/row1/rgb_images/"

# Loop through all the png images in the directory and convert them
# to jpg. Note: The order of conversion is not necessary the same
# order as how the png images were ordered.
for filename in os.listdir(directory):
    print(filename)
    if filename.endswith(".png"): 
        im = Image.open(os.path.join(directory, filename))
        rgb_im = im.convert('RGB')
        # The following line can be adjusted so that the numbering of the resulting jpg
        # images are the same as how the png images were numbered
        rgb_im.save(r"/home/aspa2/rockmelon/MMXXII/row1/png2jpg/rgb_image%s.jpg" %(num))
        num+=1
    else:
        continue
