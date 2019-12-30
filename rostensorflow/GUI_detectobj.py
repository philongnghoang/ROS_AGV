import sys
import os
from Tkinter import *
from tkMessageBox import showinfo

window=Tk()

window.title("Running Python Script")
window.geometry('550x200')

def run():
    os.system('python result2goal.py')
def run_view():
    os.system('roslaunch view_frame.launch')
def run_obj_detect():    
    os.system('python image_recognition.py')
def run_go():    
    os.system('python go_to_goal.py')
btn = Button(window, text="Getdata", bg="black", fg="white",command=run)
btn.grid(column=0, row=0)
btn = Button(window, text="View", bg="black", fg="white",command=run_view)
btn.grid(column=2, row=0)
btn = Button(window, text="Object", bg="black", fg="white",command=run_obj_detect)
btn.grid(column=4, row=0)
btn = Button(window, text="GO", bg="black", fg="white",command=run_go)
btn.grid(column=0, row=2)
window.mainloop()