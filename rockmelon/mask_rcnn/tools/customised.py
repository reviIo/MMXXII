#!/usr/bin/env/python
from tkinter import *
from tkinter import filedialog
from PIL import Image, ImageEnhance, ImageTk

# Two problems:
# 1. The image doesn't show when opened until the slider is moved
# 2. Change the starting the position of the slider

def open_image():
    global img
    try:
        img = Image.open(
            filedialog.askopenfilename(title="Select file", filetypes=(("png files", "*.png"), ("jpeg files", "*.jpg"), ("all files", "*.*"))))
        save_button.config(bg=default_color)
        brightness_slider.config(bg=default_color)
        contrast_slider.config(bg=default_color)
        sharpness_slider.config(bg=default_color)
        color_slider.config(bg=default_color)
        update_image(open_bool=True)
    except:
        pass

def save():
    global img
    if img:
        ext = StringVar()
        name = filedialog.asksaveasfilename(initialfile="Untitled", title="Select file", typevariable=ext, filetypes=(
            ('JPEG', ('*.jpg', '*.jpeg', '*.jpe')), ('PNG', '*.png'), ('GIF', '*.gif')))
        if name:
            img.save(name + "." + ext.get().lower())  # splice the string and the extension.


def change_brightness(var):
    global img, brightness_val
    brightness_val =int(var)
    update_image(brightness_bool=True)

def change_contrast(var):
    global img, contrast_val
    contrast_val =int(var)
    update_image(contrast_bool=True)

def change_sharpness(var):
    global img, sharpness_val
    sharpness_val =int(var)
    update_image(sharpness_bool=True)

def change_color(var):
    global img, color_val
    color_val =int(var)
    update_image(color_bool=True)

def update_image(brightness_bool=False, contrast_bool=False, sharpness_bool=False, color_bool=False, open_bool=False):
    global img, adapted_img, brightness_val, contrast_val, sharpness_val, color_val
    
    try:
        if brightness_bool==True:
            brightness = ImageEnhance.Brightness(img)
            imgMod = brightness.enhance((brightness_val-50)/25. +0.5)
        if contrast_bool==True:
            contrast = ImageEnhance.Contrast(img)
            imgMod = contrast.enhance((contrast_val-50)/25. +0.5)
        if sharpness_bool==True:
            sharpness = ImageEnhance.Sharpness(img)
            imgMod = sharpness.enhance((sharpness_val-50)/25. +0.5)
        if color_bool==True:
            color = ImageEnhance.Color(img)
            imgMod = color.enhance((color_val-50)/25. +0.5)
        if open_bool==True:
            imgMod = img
    except:
        pass

    adapted_img = ImageTk.PhotoImage(imgMod)
    image_container.create_image(0, 0, image=adapted_img, anchor=NW)

if __name__=="__main__":
    root = Tk()
    root.title("Image Editor")
    root.geometry('640x550')
    default_color = root.cget('bg')

    img = None
    brightness_val = 50
    contrast_val = 50
    sharpness_val = 50
    color_val = 50

    image_container = Canvas(root, borderwidth=5, relief="groove", width=640, height=480)
    image_container.pack(fill="x", expand="yes", anchor='nw', side=BOTTOM)

    open_button = Button(text='Open', font=('Arial', 20), command=open_image)
    open_button.pack(anchor='nw', side=LEFT)
    save_button = Button(text='Save', font=('Arial', 20), command=save, bg="gray")
    save_button.pack(anchor='nw', side=LEFT)

    brightness_slider = Scale(from_=0, to=100, label="Brightness", orient=HORIZONTAL, bg="gray", command=change_brightness)
    brightness_slider.pack(anchor='nw', side=LEFT)
    contrast_slider = Scale(from_=0, to=100, label="Contrast", orient=HORIZONTAL, bg="gray", command=change_contrast)
    contrast_slider.pack(anchor='nw', side=LEFT)
    sharpness_slider = Scale(from_=0, to=100, label="Sharpness", orient=HORIZONTAL, bg="gray", command=change_sharpness)
    sharpness_slider.pack(anchor='nw', side=LEFT)
    color_slider = Scale(from_=0, to=100, label="Color", orient=HORIZONTAL, bg="gray", command=change_color)
    color_slider.pack(anchor='nw', side=LEFT)

    root.mainloop()