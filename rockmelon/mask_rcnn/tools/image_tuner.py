#!/usr/bin/env python
import os
import cv2
from tkinter import *
from PIL import Image, ImageEnhance

def tuner(var, img):
    converter = ImageEnhance.Color(img)
    result = converter.enhance(var)

    result.show()

'''def slider():
    cv2.getTrackbarPos("Brightness", "Image Tuner")
    cv2.getTrackbarPos("Contrast", "Image Tuner")
    cv2.getTrackbarPos("Sharpness", "Image Tuner")
    cv2.getTrackbarPos("Color", "Image Tuner")'''

if __name__=="__main__":
    #filename = input(r"Enter the filename: ")
    filename = "rgb_image1.png"
    filename = os.path.join(r"/home/marvin/mlfolder/MMXXII/Dataset/train", filename)
    img = Image.open(filename)
    '''cv2.namedWindow("Image Tuner")
    cv2.createTrackbar("Brightness", "Image Tuner", 0, 1000, tuner)
    cv2.createTrackbar("Contrast", "Image Tuner", 0, 1000, tuner)
    cv2.createTrackbar("Sharpness", "Image Tuner", 0, 1000, tuner)
    cv2.createTrackbar("Color", "Image Tuner", 0, 1000, tuner)'''
    # tuner(img)
    root = Tk()
    var = DoubleVar()
    root.title("Image Tuner")

    open_button = Button(text='Open Image', font=('Arial', 20), command=open_image)
    open_button.pack(anchor='nw', side=LEFT)

    # Create a canvas
    canvas= Canvas(root, width=1024, height=768)
    canvas.pack()

    # Set up the the slider
    contrast_slider = Scale(root, from_=0, to=1000, length=200, orient=HORIZONTAL, bg="orange", troughcolor="yellow", command=tuner)
    contrast_slider.pack(anchor=CENTER)
    #cv2.waitKey(0)

    root.mainloop()