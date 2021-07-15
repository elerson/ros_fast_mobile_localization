#!/usr/bin/python
import numpy as np
import math
import sys

f = open(sys.argv[1], "r")
size = 600
resolution = 0.025
current_line = f.readline()
while current_line:

    data = current_line.split(',')
    
    id_ = int(data[0])
    x = (float(data[1])+size)*resolution
    y = (float(data[2])+size)*resolution
    print(f'{id_}: x -> {x:.02f} y -> {y:.02f}')
    
    current_line = f.readline()
