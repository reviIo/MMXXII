#!/usr/bin/env python
from math import gamma
import os
import cv2

'''img = cv2.imread("/home/aspa2/rockmelon/MMXXII/Dataset/test/rgb_image1.jpg")
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("Grayscale Image", img_gray)
cv2.waitKey(0)
cv2.destroyAllWindows()'''

def BrightnessContrast(brightness=0):
    # getTrackbarPos returns the current position of the sprcified trackbar
    brightness = cv2.getTrackbarPos('Brightness', 'Image Tuner')
    contrast = cv2.getTrackbarPos('Contrast', 'Image Tuner')

    effect = controller(img, brightness, contrast)

    # The function imshow displays an image in the specified window
    cv2.imshow('Effect', effect)

def controller(img, brightness=255, contrast=127):
    brightness = int((brightness - 0) * (255 - (-255)) / (510 - 0) + (-255))
    contrast = int((contrast - 0) * (127 - (-127)) / (254 - 0) + (-127))

    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            max = 255
        else:
            shadow = 0
            max = 255 + brightness

        al_pha = (max - shadow) / 255
        ga_mma = shadow
        
        # The function addWeighted calculates the weighted sum fo two arrays
        cal = cv2.addWeighted(img, al_pha, img, 0, ga_mma)

    else:
        cal = img
    
    if contrast != 0:
        Alpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
        Gamma = 127 * (1- Alpha)

        # The function addWeighted calculates the weighted sum of two arrays
        cal = cv2.addWeighted(cal, Alpha, cal, 0, Gamma)
    
    # putText renders the spicified text string in the image.
    cv2.putText(cal, 'B:{},C:{}'.format(brightness, contrast), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,225), 2)

    return cal

if __name__=="__main__":
    filename = input("Enter the filename: ")

    # The function imread loads an image from the specified file and returns it
    original = cv2.imread(os.path.join("/home/aspa2/rockmelon/MMXXII/Dataset/test", filename))

    # Make another copy of an image
    img = original.copy()

    # The function namedWindow created a window that can be used as a placeholder for images
    cv2.namedWindow('Image Tuner')

    # The function imshow displays an image in the specifed window
    cv2.imshow('Image Tuner', original)

    # createTrackbar(trackbarName, windowName, value, count, onChange)
    # Brightness range -255 to 255
    cv2.createTrackbar('Brightness', 'Image Tuner', 255, 2 * 255, BrightnessContrast)

    # Contrast range -127 to 127
    cv2.createTrackbar('Contrast', 'Image Tuner', 127, 2 * 127, BrightnessContrast)

    BrightnessContrast(brightness=0)

    # The function waitKey waits for a key event infinitely or for delay miliseconds, when it is position
    cv2.waitKey(0)