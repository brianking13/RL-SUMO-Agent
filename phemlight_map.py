# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 09:42:10 2023

@author: bking
"""

import numpy as np
import matplotlib.pyplot as plt
import csv
import numpy as np
from collections import OrderedDict

# reading csv file

filename = "PHEMLIGHT_map.csv"
fields = []
rows = []

with open(filename, 'r') as csvfile:
    # creating a csv reader object
    csvreader = csv.reader(csvfile, delimiter=';')
     
    # extracting field names through first row
    fields = next(csvreader)
 
    # extracting each data row one by one
    for row in csvreader:
        rows.append(row)
 
    # get total number of rows
    print("Total no. of rows: %d"%(csvreader.line_num))
    

velocity = [float(item[0]) for item in rows if item[3]=='fuel' if item[2] == '5']
accel = [float(item[1]) for item in rows if item[3]=='fuel' if item[2] == '5'] 
fuel = [float(item[4]) for item in rows if item[3]=='fuel' if item[2] == '5']
    


unique_velocity= list(OrderedDict.fromkeys(velocity))
unique_accel= list(OrderedDict.fromkeys(accel))


# create heat map
Y,X = np.meshgrid(unique_accel,unique_velocity)
fuel = np.asarray(fuel)
Z=fuel.reshape(len(unique_velocity),len(unique_accel))

plt.pcolormesh(X,Y,Z,shading='gouraud',cmap = "jet")
plt.colorbar(label = 'Fuel Consumption (mL/s)')
plt.xlabel('Speed (m/s)')
plt.ylabel('Acceleration (m/s^2)')

# plt.show()