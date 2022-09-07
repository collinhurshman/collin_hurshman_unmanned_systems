# -*- coding: utf-8 -*-
"""
Created on Wed Aug 24 10:01:27 2022

@author: colli
"""

import matplotlib.pyplot as plt
import numpy as np

def getNode(x,y,Xmax,spacing):
    #returns node index from an X,Y pair assuming index increases across each row from the left...
    # starting at the bottom row, with 0 index included
    rowOffset = Xmax/spacing + 1 #how many indices are in each row, including starting at 0...
    return 1*x/spacing + y*rowOffset/spacing#note this requires spacing to be identical on x,y

Xmax = 10
Ymax = 10
spacing = 0.5
xSpan = np.arange(0,Xmax+spacing,spacing)#make sure to include Xmax by adding 1 extra spacing
ySpan = np.arange(0,Ymax+spacing,spacing)

for i in range(len(xSpan)):#apply the node eqn and plot each node number... row by row
    x = xSpan[i]
    #now need each unique y value corresponding to each x
    for z in range(len(ySpan)):
        y = ySpan[z]
        plt.text(x,y,str(int(getNode(x,y,Xmax,spacing))),color="red",fontsize = 8)

plt.axis([0,Xmax+spacing,0,Ymax+spacing])#set x, y axes from 0 to 10 with some padding for the text to fit
plt.title('Node Indices')
plt.show()
#just testing at some point...
print(getNode(8,4,10,0.5))
val = getNode(8,4,10,0.5)
assert val == 183