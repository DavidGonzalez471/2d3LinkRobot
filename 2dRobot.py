import numpy as np 
import matplotlib.pyplot as plt 
import math
import time
import pandas as pd


#Link Length
link = [150,100,50]
#Initial Joint angles
angle = [90, 0, 0]
# Target Position using main target as well to maintain the target value
main_target = [0, 0]
target = [0, 0]
hand=[0,300]

table_vals=[]


# Create figure to plot
fig = plt.figure(1) 
ax = fig.add_subplot(1,1,1)
#create figure for the table 
fig2,ax2 = plt.subplots(figsize=(10,10))


#create background with a grid at a spacing of every 20 units.
def background():
    major_ticks = np.arange(-300, 300, 20)
    circle = np.linspace(0,2 *np.pi, 150)
    radius = 300
    a = radius * np.cos(circle)
    b = radius * np.sin(circle)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)

    ax.axis([-300,300, 0, 300])
    ax.grid(which='major', alpha= 1)
    ax.plot(a, b)
    ax.set_aspect( 1 )
