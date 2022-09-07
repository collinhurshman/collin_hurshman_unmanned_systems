# -*- coding: utf-8 -*-
"""
Created on Wed Aug 24 09:45:43 2022

@author: colli
"""
import numpy as np
import matplotlib.pyplot as plt

x = list(np.arange(0,10))
y = list(np.arange(10,20))
for i in range(0,len(x)):
    print("x is ",x[i])
    print("y is " ,y[i])
    
    
print("\n")

for itemx,itemy in zip(x,y):
    print(itemx)
    print(itemy)

for i, (itemx,itemy) in enumerate(zip(x,y)):
    print(i,itemx,itemy)

plt.plot(x,y)

def add(x:float,y:float) -> float:#don't HAVE to specify datatypes.. but can
    #only works in python 3 +    
    return x*y