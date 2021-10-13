#!/usr/bin/python
import numpy as np
import math
import sys

fp = open(sys.argv[1], "r")
fs = open(sys.argv[2], "r")
size = 600
resolution = 0.025
current_line = fp.readline()

sensor_positions = {}
while current_line:

    data = current_line.split(',')
    
    id_ = int(data[0])
    x = (float(data[1])+size)*resolution
    y = (float(data[2])+size)*resolution
    sensor_positions[id_] = (x, y)
        
    current_line = fp.readline()

current_line = fs.readline()
while current_line:

    data = current_line.split(',')
    
    id_ = int(data[0])
    x = (float(data[1]))*resolution
    y = (float(data[2]))*resolution
    
    w = (float(data[3]))*resolution
    h = (float(data[4]))*resolution
    
    s_id_ = int(data[5])
    
    print(f'{id_}: [[{x:.02f}, {y:.02f}, {w:.02f}, {h:.02f}], [{sensor_positions[s_id_][0]:.02f}, {sensor_positions[s_id_][1]:.02f}]]')    
 
    current_line = fs.readline()
