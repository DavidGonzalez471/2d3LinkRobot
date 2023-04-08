import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
import math
import pandas as pd
import Table as tb


#Link Length
link = [150,100,50]
#Initial Joint angles
angle = [90, 0, 0]
# Target Position using main target as well to maintain the target value
main_target = [0, 0]
target = [0, 0]
hand=[0,300]


# Create figure to plot
fig = plt.figure(1) 
ax = fig.add_subplot(1,1,1)



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
    ax.set_aspect(1)

'''pythhagorean theorem
if you have two sides available you can use this to calculate the other side.'''
def pythag(a,b,c):
    if not c:
        c = math.sqrt(a**2 + b**2)
        return c
    
#indicating the tilt of the links 
def movement(theta):
    m = np.array([[math.cos(theta), - math.sin(theta), 0],
                   [math.sin(theta), math.cos(theta), 0],
                   [0, 0, 1]])
    return m

def main():
    #event for mouseclick, setting limits of the graph and creating the starting position.
    fig.canvas.mpl_connect('button_press_event', onclick)
    ax.set_xlim(-300, 300)
    ax.set_ylim(0, 300)

    # Forward Kinematics
    P = FK(angle, link)

    ax.plot([0,0],[0,150],
            [0,0], [150,250],
            [0,0], [250,300] )
    start_point = [0,300]
    background()

    plt.show()

if __name__ == "__main__":
    main()
