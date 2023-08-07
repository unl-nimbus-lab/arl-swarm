#Stupid python script that plost rectangles because c++ can't easily

import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

with open('/home/grant/myrs.json') as file:
    j = json.load(file)

numberOfRects = j[0]["Rectangles"]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def plotRect(idx, jsonthing, axHandle):
    #Plot a rectangle from a particular index

    #Step 1) Get the intervals from the json file
    dimensions = jsonthing[idx]["Dimension"]
    intervalss = []
    for dim in range(dimensions):
        intervalss.append( jsonthing[idx]["Intervals"][dim] )

    #Step 2) create the vertices of the rectangle
    vertices = []

    for x in intervalss[0]:
        for y in intervalss[1]:
            for z in intervalss[2]:
                vertices.append([x,y,z])

    #Step 3) add the edges to the plot
    for x in intervalss[0]:
        for y in intervalss[1]:
            xs = [x,x]
            ys = [y,y]
            zs = [intervalss[2][0], intervalss[2][1]]
            axHandle.plot(xs,ys,zs,'b')

    for y in intervalss[1]:
        for z in intervalss[2]:
            xs = [intervalss[0][0], intervalss[0][1]]
            ys = [y,y]
            zs = [z,z]
            axHandle.plot(xs,ys,zs,'b')

    for x in intervalss[0]:
        for z in intervalss[2]:
            xs = [x,x]
            ys = [intervalss[1][0], intervalss[1][1]]
            zs = [z,z]
            axHandle.plot(xs,ys,zs,'b')


for rect in range(1,numberOfRects + 1):
    plotRect(rect,j,ax)


# Set plot limits and labels
ax.set_xlim([-100, 100])
ax.set_ylim([-100, 100])
ax.set_zlim([300, 400])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
