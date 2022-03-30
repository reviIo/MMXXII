#!/usr/bin/env python
import os
import cv2
from PIL import Image, ImageEnhance

def BrightnessContrast(brightness=0):
 
    # getTrackbarPos returns the
    # current position of the specified trackbar.
    brightness = cv2.getTrackbarPos('Brightness', 'Image Tuner')
    contrast = cv2.getTrackbarPos('Contrast', 'Image Tuner')
    sharpness = cv2.getTrackbarPos('Sharpness', 'Image Tuner')
    color = cv2.getTrackbarPos('Color', 'Image Tuner')

    effect = controller(img,
                        brightness,
                        contrast,
                        sharpness,
                        color)
 
    # The function imshow displays
    # an image in the specified window
    cv2.imshow('Effect', effect)

def controller(img, brightness=1, contrast=1, sharpness=1, color=1):
    filter = ImageEnhance.Brightness(img)
    new_image = img.filter(brightness)

    filter = ImageEnhance.Contrast(img)
    new_image = img.filter(contrast)

    filter = ImageEnhance.Sharpness(img)
    new_image = img.filter(sharpness)

    filter = ImageEnhance.Color(img)
    new_image = img.filter(color)
 
    return new_image

if __name__ == '__main__':
    filename = input("Enter the filename: ")

    filename = os.path.join("/home/aspa2/rockmelon/MMXXII/Dataset_Old/train", filename)
    img = Image.open(filename)
    img.show()

    # The function namedWindow creates
    # a window that can be used as
    # a placeholder for images.
    cv2.namedWindow('Image Tuner')

    # createTrackbar(trackbarName,
    # windowName, value, count, onChange)
    # Brightness range 0 to 1000
    cv2.createTrackbar('Brightness', 'Image Tuner',
                       0, 1000,
                       BrightnessContrast)
     
    # Contrast range 0 to 1000
    cv2.createTrackbar('Contrast', 'Image Tuner',
                       0, 1000,
                       BrightnessContrast) 
    
    # Sharpness range 0 to 1000
    cv2.createTrackbar('Sharpness', 'Image Tuner',
                       0, 1000,
                       BrightnessContrast) 
    
    # Color range 0 to 1000
    cv2.createTrackbar('Color', 'Image Tuner',
                       0, 1000,
                       BrightnessContrast) 
    
    BrightnessContrast(0)