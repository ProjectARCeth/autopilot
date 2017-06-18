import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np
import pylab

def extractSmallValues(value, limit):
	if abs(value) < limit:
		return value
	else:
		return 0


# Read map file and delete [] and ;.
coords_map_file = open("/home/sele/.ros/MapPoints.txt", "r")
infile = "/home/sele/.ros/MapPoints.txt"
outfile = "/home/sele/.ros/MapPointsCleared.txt"

delete_list = ["]", "[", ";", " "]
fin = open(infile)
fout = open(outfile, "w+")
for line in fin:
    for word in delete_list:
        line = line.replace(word, "")
    fout.write(line)
fin.close()
fout.close()

# Creating coords list and remove cleared file.
coords_map_file = open(outfile)
coords = []
for line in coords_map_file:
	cleared_line = line.replace("\n", "")
	coords.append(cleared_line)
coords_map_file.close()
os.remove(outfile)

# Assigning coordinates.
x = []
y = []
z = []
limit = 100
for i in range(0, len(coords)-3):
	x.append(extractSmallValues(float(coords[i]), limit))
	y.append(extractSmallValues(float(coords[i+1]), limit))
	z.append(extractSmallValues(float(coords[i+2]), limit))
	i = i+3

# Plotting.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c='r')
pylab.savefig('map.png')