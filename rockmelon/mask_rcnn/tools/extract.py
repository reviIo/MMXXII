#!/usr/bin/env python
from tkinter import *
from tkinter import filedialog
from PIL import Image, ImageEnhance, ImageTk


def update_image():
    global img, adapted_img,contrast_val
    contrast = ImageEnhance.Contrast(img)
    imgMod = contrast.enhance((contrast_val-50)/25. +0.5)

    adapted_img = ImageTk.PhotoImage(imgMod)
    image_container.create_image(0, 0, image=adapted_img, anchor=NW)


def open_image():
    global img
    try:
        img = Image.open(
            filedialog.askopenfilename(title="Select file", filetypes=(("jpeg files", "*.jpg"), ("all files", "*.*"))))
        save_button.config(bg=default_color)
        flip_horizontal_button.config(bg=default_color)
        flip_vertical_button.config(bg=default_color)
        contrast_slider.config(bg=default_color)
        update_image()
    except:
        pass


def flip_horizontal():
    global img
    if img:
        img = img.transpose(Image.FLIP_LEFT_RIGHT)
        update_image()


def flip_vertical():
    global img
    if img:
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        update_image()


def save():
    global img
    if img:
        ext = StringVar()
        name = filedialog.asksaveasfilename(initialfile="Untitled", title="Select file", typevariable=ext, filetypes=(
            ('JPEG', ('*.jpg', '*.jpeg', '*.jpe')), ('PNG', '*.png'), ('GIF', '*.gif')))
        if name:
            img.save(name + "." + ext.get().lower())  # splice the string and the extension.


def change_contrast(var):
    global img, contrast_val
    contrast_val =int(var)
    update_image()


root = Tk()
root.title("Image Editor")
root.geometry('600x500')
default_color = root.cget('bg')

img = None
contrast_val = 50

open_button = Button(text='Open Image', font=('Arial', 20), command=open_image)
flip_horizontal_button = Button(text='Flip Horizontal', font=('Arial', 10), command=flip_horizontal, bg="gray",
                                width=15)
flip_vertical_button = Button(text='Flip Vertical', font=('Arial', 10), command=flip_vertical, bg="gray", width=15)
contrast_slider = Scale(from_=0, to=100, orient=HORIZONTAL, bg="gray", command=change_contrast)
save_button = Button(text='Save', font=('Arial', 20), command=save, bg="gray")
image_container = Canvas(root, borderwidth=5, relief="groove", width=300, height=300)

image_container.pack(fill="both", expand="yes", anchor='nw', side=BOTTOM)
open_button.pack(anchor='nw', side=LEFT)
save_button.pack(anchor='nw', side=LEFT)
contrast_slider.pack(anchor='w', side=LEFT)
flip_horizontal_button.pack(anchor='w')
flip_vertical_button.pack(anchor='w')

root.mainloop()