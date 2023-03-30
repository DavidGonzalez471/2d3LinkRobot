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

#Values for the table
Bpos=[0,0]
AB_angle = 0
AB_angle_diff = 0
Cpos=[0,0]
BC_angle=[0,0]
BC_angle_diff=[0,0]
Dpos=[0,0]
CD_angle=0
CD_angle_diff=0
Max_angle_diff=0
