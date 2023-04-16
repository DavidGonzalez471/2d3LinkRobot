import numpy as np 
import matplotlib.pyplot as plt 
import math
import pandas as pd


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
#indicating the movement of the links
def translate(dx, dy):
    t = np.array([[1, 0, dx],
                  [0, 1, dy],
                  [0, 0, 1]])
    return t

# Forward Kinematics
# Input initial angles and length of links
# Output positions each points
def FK(angle, link):
    
    P = []
    P.append(np.eye(3))
    for i in range(len(link)):
        M = movement(angle[i]/180*math.pi)
        T = translate(link[i], 0)
        P.append(P[-1].dot(M).dot(T))
    return P
'''Inverse Kinematic function to guide the end effector to the target point, the inverse kinematic function finds the angle of the joints
    and iteratively finds the optimal solution'''
def IK(target, angle, link, max_iter = 1000, err_min = 0.1):
    global Bpos, AB_angle, AB_angle_diff, Cpos, BC_angle, BC_angle_diff, Dpos, CD_angle, CD_angle_diff, Max_angle_diff
    solved = False
    max_angle =10
    angle_temp = 0
    err_end_to_target = math.inf
    
    for loop in range(max_iter):
        for i in range(len(link)-1, -1, -1):
            P = FK(angle, link)
            end_to_target = target - P[-1][:2, 2]
            err_end_to_target = math.sqrt(end_to_target[0] ** 2 + end_to_target[1] ** 2)

            if err_end_to_target < err_min:
                if i == 0:
                    angle_temp += abs(angle[i] - 90)
                    if angle_temp < max_angle:
                        solved = True
                if angle[i] > 180:
                    angle_temp = angle_temp + (abs(angle[i]-360))
                else:
                    angle_temp = angle_temp + (abs(angle[i]))
    

            
            # Calculate distance joint and end effector
            # P[i] is position of current joint
            # P[-1] is position of end effector
            cur_to_end = P[-1][:2, 2] - P[i][:2, 2]
            cur_to_end_mag = pythag(cur_to_end[0], cur_to_end[1], None)
            cur_to_target = target - P[i][:2, 2]
            cur_to_target_mag = pythag(cur_to_target[0], cur_to_target[1], None)

            end_target_mag = cur_to_end_mag * cur_to_target_mag

            if end_target_mag <= 0.0001:    
                cos_rot_ang = 1
                sin_rot_ang = 0
            else:
                cos_rot_ang = (cur_to_end[0] * cur_to_target[0] + cur_to_end[1] * cur_to_target[1]) / end_target_mag
                sin_rot_ang = (cur_to_end[0] * cur_to_target[1] - cur_to_end[1] * cur_to_target[0]) / end_target_mag

            rot_ang = np.arccos(max(-1, min(1,cos_rot_ang)))

            if sin_rot_ang < 0.0:
                rot_ang = -rot_ang


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
