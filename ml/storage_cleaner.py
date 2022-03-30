""" This code keeps all the most recent 15 weights and delete all the 
older weights except the ones that are a multiple of 200. This is done
to save space."""

#!/usr/bin/env python
import os
import sys
import time
from shutil import copyfile

# Specify the path to the directory where the weights will be stored
directory = "/media/aspa2/KINGSTON/SummerResearch/MMXXII/logs"

files = os.listdir(directory)
sorted_files = sorted(files, key=lambda x: int(x[6:14]*10000+x[15:]))

print(sorted_files)

try:
    while True:
        # Create a list of files in the directory
        model_list = []
        target = sorted_files[-1]
        folder = os.path.join(directory, target)
        models = os.listdir(folder)
        for model in models:
            # Fiter filenames to find the files that is a weight file
            if model.startswith("mask_rcnn_object"):
                answer = int(model[17:21]) % 200
                if answer == 0:
                    # All the filenames with number that are a multiple of 200 are thrown
                    # into the "archives" directory. If the directory does not exist, 
                    # a new one is created. 
                    destination = os.path.join(folder, "archives")
                    if os.path.exists(destination)==False:
                        os.mkdir(destination)
                    # Copy the weight file into the "archives" directory
                    copyfile(os.path.join(folder, model), os.path.join(destination, model))
                model_list.append(model)
        
        # Check for the number in the filename and using list indices to remove the files 
        # that are not the most recent 15
        model_list = sorted(model_list, key=lambda x: int(x[17:21]))
        while len(model_list) > 15:
            os.remove(os.path.join(folder, model_list[0]))
            model_list=model_list[1:]
        print(model_list)
        # Check for new files every 30 seconds
        time.sleep(30)
except KeyboardInterrupt:
    sys.exit(0)