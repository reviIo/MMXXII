#!/usr/bin/env python
import os
import sys
import time
from shutil import copyfile

directory = "/media/aspa2/KINGSTON/SummerResearch/MMXXII/logs"

files = os.listdir(directory)
sorted_files = sorted(files, key=lambda x: int(x[6:14]*10000+x[15:]))

print(sorted_files)

try:
    while True:
        model_list = []
        target = sorted_files[-1]
        folder = os.path.join(directory, target)
        models = os.listdir(folder)
        for model in models:
            if model.startswith("mask_rcnn_object"):
                answer = int(model[17:21]) % 200
                if answer == 0:
                    destination = os.path.join(folder, "archives")
                    if os.path.exists(destination)==False:
                        os.mkdir(destination)
                    copyfile(os.path.join(folder, model), os.path.join(destination, model))
                model_list.append(model)
        model_list = sorted(model_list, key=lambda x: int(x[17:21]))
        while len(model_list) > 15:
            os.remove(os.path.join(folder, model_list[0]))
            model_list=model_list[1:]
        print(model_list)
        time.sleep(30)
except KeyboardInterrupt:
    sys.exit(0)
