# -*- coding: utf-8 -*-
"""
Created on Wed Aug 31 09:34:29 2022

@author: colli
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Aug 29 13:00:53 2022

@author: colli
"""
import matplotlib.pyplot as plt
import numpy as np

class node:
    def __init__(self,x,y,cost,parent_index):
        #create variables unique to each particular instance of the node class, 
        #using the input parameters
        self.X = x
        self.Y = y
        self.cost = cost
        self.parent_index = parent_index

class grid:
    def __init__(self,Xmax,Ymax,spacing):#note this grid always starts at node 0
        self.Xmax = Xmax
        self.Ymax = Ymax
        self.spacing = spacing
        
def getNode(x,y,Xmax,spacing):
    #returns node index from an X,Y pair assuming index increases across each row from the left...
    # starting at the bottom row, with 0 index included
    rowOffset = Xmax/spacing + 1 #how many indices are in each row, including starting at 0...
    return 1*x/spacing + y*rowOffset/spacing#note this requires spacing to be identical on x,y

def distance(p1,p2):
    distance = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(1/2) #note ** is used for power to distinguish 
    #from bitwise ops...
    #above line simply computes the euclidean distance between two points where args are two X,Y pairs
    return distance

def checkValidity(grid, cost, nodeObj,index, obsCoords,visited):
    #returns true if node is valid, false otherwise
    flag = True
    obsIndices = []
    #!!!UPDATE THIS TO INCLUDE ROBOT RADIUS
    for obs in obsCoords:
        obsIndices.append(getNode(obs[0],obs[1],grid.Xmax,grid.spacing))
    if index in obsCoords or index in visited:#ignore visited nodes or those that are obstacles
        flag = False
    if cost == 0: #ignore the center node... MOVE THIS TO A SEPARATE CONDITION
        flag = False
    if nodeObj.X < 0 or nodeObj.X > grid.Xmax:#check against size of grid
        flag = False
    if nodeObj.Y < 0 or nodeObj.Y > grid.Ymax:
        flag = False
    return flag
    
    
def dijkstra(startCoords, endCoords, obsCoords, grid):
    #all done... now show the result
    plt.axis([0,grid.Xmax+grid.spacing,0,grid.Ymax+grid.spacing])#set x, y axes from 0 to 10 with padding for the text to fit
    plt.title('Dijkstra Shortest Path & Costs')
    plt.show()
    
    unvisited = {}
    visited = {} 

    currentNode = node(startCoords[0],startCoords[1],0,-1)
    currentIndex = getNode(startCoords[0],startCoords[1],grid.Xmax,grid.spacing)
    unvisited[currentIndex] = currentNode
    
    while len(unvisited):#iterate as long as there are unvisited nodes...
        #note that in some cases, it may be better to stop as soon as the end is reached...
        currentIndex = min(unvisited,key=lambda x:unvisited[x].cost)
        currentNode = unvisited[currentIndex]
        visited[currentIndex] = unvisited[currentIndex]
        del unvisited[currentIndex]#remove the current node from unvisited...
        #only plot each node after lowest cost is known, ie the node is visited
        plt.text(currentNode.X,currentNode.Y,str('%.2f' % float(currentNode.cost)),color = "red",fontsize = 6)#only show a couple decimal places in cost
        #evaluate the neighbors...
        for i in [-grid.spacing, 0, grid.spacing]:
            for j in [-grid.spacing,0,grid.spacing]:        
                tempX = currentNode.X + i
                tempY = currentNode.Y + j
                tempNode = node(tempX,tempY,0,currentIndex)#use current node index as parent index...
                tempIndex = getNode(tempNode.X,tempNode.Y,grid.Xmax,grid.spacing)
                costCalc = distance([currentNode.X,currentNode.Y],[tempNode.X,tempNode.Y])
                tempNode.cost = currentNode.cost + costCalc

                #only update dictionaries if the node provides a new, viable path        
                
                if checkValidity(grid, costCalc,tempNode,tempIndex,obsCoords,visited): #make sure the location is valid and not hte 
                    
                    if tempIndex in unvisited:#already looked at this node... is there a shorter path to it?
                        if tempNode.cost < unvisited[tempIndex].cost:
                            tempNode.parent_index = currentIndex
                    else:
                        tempNode.parent_index = currentIndex
                        unvisited[tempIndex] = tempNode #update unvisited if we discover a new unvisited node...
    
       
                        
myGrid = grid(10,10,0.5)
startCoords = [0,0]
endCoords = [10,10]
obsCoords = []
dijkstra(startCoords,endCoords,obsCoords,myGrid)

