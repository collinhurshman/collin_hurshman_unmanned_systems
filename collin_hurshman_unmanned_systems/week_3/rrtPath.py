# -*- coding: utf-8 -*-
"""
Created on Fri Sep  2 10:22:42 2022

@author: colli
"""

# -*- coding: utf-8 -*-

"""
Created on Mon Aug 29 13:00:53 2022

@author: colli
"""

import matplotlib.pyplot as plt
import numpy as np
import random
import math

class Node:
    def __init__(self,x,y,cost,parent_index):
        #create variables unique to each particular instance of the node class, 
        #using the input parameters
        self.X = x
        self.Y = y
        self.cost = cost
        self.parent_index = parent_index
        self.estCost = cost

class Grid:
    def __init__(self,Xmax,Ymax,spacing):#note this grid always starts at node 0
        self.Xmax = Xmax
        self.Ymax = Ymax
        self.spacing = spacing
        

class rrtPath():
    def __init__(self,grid:Grid,obsList:list,start:tuple,goal:tuple,robotRadius:float):
        self.grid = grid
        self.obsList = obsList
        self.start = start
        self.end = goal
        self.radius = robotRadius
        self.obsRadius = self.grid.spacing/2 
        self.tolerance = self.grid.spacing
        self.path = []
        self.cost = 0
        self.stepDist = self.grid.spacing
    
    def getNode(self,x,y,Xmax,spacing):
        #returns node index from an X,Y pair assuming index increases across each row from the left...
        # starting at the bottom row, with 0 index included
        rowOffset = Xmax/spacing + 1 #how many indices are in each row, including starting at 0...
        return 1*x/spacing + y*rowOffset/spacing#note this requires spacing to be identical on x,y
    
    def distance(self,p1,p2):
        dist = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(1/2) #note ** is used for power to distinguish 
        #from bitwise ops...
        #above line simply computes the euclidean distance between two points where args are two X,Y pairs
        return dist

    def showPlots(self):
        plt.show()

    def __checkValidity(self, nodeObj): #a private class method for checking validity of nodes
        #returns true if node is valid, false otherwise
        flag = True
        for obs in self.obsList:
            if self.distance(obs,(nodeObj.X,nodeObj.Y)) - self.radius  <= self.obsRadius:
                #notice the robot IS allowed to rub right against the obstacles
                flag = False
        if nodeObj.X < self.radius or nodeObj.X > self.grid.Xmax - self.radius:#check against size of grid
            flag = False
        if nodeObj.Y < self.radius or nodeObj.Y > self.grid.Ymax - self.radius:
            flag = False
        return flag
        
    def showBaseGrid(self):
        plt.figure()
        plt.axis([0,self.grid.Xmax+ self.grid.spacing,0,self.grid.Ymax+self.grid.spacing])#set x, y axes from 0 to 10 with padding for the text to fit
        plt.title('Workspace with Node Indices')
        xSpan = np.arange(0,self.grid.Xmax+self.grid.spacing,self.grid.spacing)#make sure to include Xmax by adding 1 extra spacing
        ySpan = np.arange(0,self.grid.Ymax+self.grid.spacing,self.grid.spacing)

        for i in range(len(xSpan)):#apply the node eqn and plot each node number... row by row
            x = xSpan[i]
            #now need each unique y value corresponding to each x
            for z in range(len(ySpan)):
                y = ySpan[z]
                colorStr = "green"
                for obs in self.obsList:
                    if x == obs[0] and y == obs[1]:#color node red if it is an obstacle
                        colorStr = "red"
                plt.text(x,y,str(int(self.getNode(x,y,self.grid.Xmax,self.grid.spacing))),color = colorStr,fontsize = 8)
   
    def findPath(self, plotDesired:bool = 1):
        nodeTree = {}
        if plotDesired: #only set up a figure if the user requests it
            plt.figure()
            plt.title("Node Obstacles, Tree, with Path Highlighted")
            plt.axis([0,self.grid.Xmax+ self.grid.spacing,0,self.grid.Ymax+self.grid.spacing])
        
        currentNode = Node(self.start[0],self.start[1],0,-1)
        index = self.getNode(self.start[0],self.start[1],self.grid.Xmax,self.grid.spacing)
        nodeTree[index] = currentNode
        while self.distance((currentNode.X,currentNode.Y), (self.end[0],self.end[1])) > self.tolerance:
            newLocal = (round(random.randrange(0,self.grid.Xmax)/self.grid.spacing)*self.grid.spacing
            ,round(random.randrange(0,self.grid.Ymax)/self.grid.spacing)*self.grid.spacing)

            nodeAndDist = {}
            for listing in nodeTree:
                node = nodeTree[listing]
                dist = self.distance(newLocal,(node.X,node.Y))
                ind = self.getNode(node.X,node.Y,self.grid.Xmax,self.grid.spacing)
                nodeAndDist[ind] = dist
            #now have a list of indices and their distance... find minimum...
            closestIndex = min(nodeAndDist,key=nodeAndDist.get)
            currentNode = nodeTree[closestIndex]
            closeLocal = (currentNode.X,currentNode.Y)
            angle = math.atan2((newLocal[1]-currentNode.Y),(newLocal[0]-currentNode.X))#decide what direction should be headed
            newX = currentNode.X + round(self.stepDist*math.cos(angle)/self.grid.spacing)*self.grid.spacing #round x,y to nearest gridspacing
            newY = currentNode.Y + round(self.stepDist*math.sin(angle)/self.grid.spacing)*self.grid.spacing
            
            cost = currentNode.cost + self.distance((currentNode.X,currentNode.Y),(newX,newY))
            parent = self.getNode(currentNode.X,currentNode.Y,self.grid.Xmax,self.grid.spacing)
            currInd = self.getNode(newX,newY,self.grid.Xmax,self.grid.spacing)
            currentNode = Node(newX,newY,cost,parent)#update the current node to the new location..
            #just to make sure it isn't already on the goal...
            if self.__checkValidity(currentNode):#only save to tree if it's valid...
                if currInd in nodeTree:
                    if cost < nodeTree[currInd].cost:
                        #if the random branch has already been explored...
                        #only update its node if its shorter than previous branch
                        #this likely only happens due to rounding...
                        #the minimum distance criterion would otherwise choose that shorter dist when selecting the node...
                        nodeTree[currInd] = currentNode
                else:
                    nodeTree[currInd] = currentNode
        self.path = []
        #final node may be close but not exactly on final goal... add the final goal to the path
        #also can add it to the nodetree
        endIndex = self.getNode(self.end[0],self.end[1],self.grid.Xmax,self.grid.spacing)    
        if newX != self.end[0] or newY != self.end[1]:   
            cost = currentNode.cost + self.distance((newX,newY),(self.end[0],self.end[1]))
            nodeTree[endIndex] = Node(self.end[0],self.end[1],cost,currInd)
            self.path.append(self.end)
            self.cost = cost
        else:#end is the same as the current...
            self.path.append((currentNode.X,currentNode.Y))
            self.cost = currentNode.cost

        parent = nodeTree[endIndex].parent_index#iterate backwards through the list
        
        while parent != -1:
            temp = nodeTree[parent]#go up to the next closest node
            parent = temp.parent_index
            self.path.append((temp.X,temp.Y))
        self.path = self.path[::-1]#reverse the path order so it travels start to finish
        
        if plotDesired:
            for nodeIdx in nodeTree:
                node = nodeTree[nodeIdx]
                plt.plot(node.X,node.Y,marker = ".",color = "orange")
                
            for i in range(0, len(self.path)-1):
                point1 = self.path[i]
                point2 = self.path[i+1]
                xvals = [point1[0], point2[0]]
                yvals = [point1[1], point2[1]]
                plt.plot(xvals,yvals, 'b.-')
                #NOW NEED TO ITERATE FROM ONE POINT TO THE NEXT, DRAWING LINE...
            for obs in self.obsList:
                plt.plot(obs[0],obs[1],marker = ".",color = "r")
        return self.path
"""
#below is purely for testing...
myGrid = Grid(10,10,0.5)
startCoords = (0,0)
endCoords = (8,9)
obsCoords = [(1,1),(4,4),(3,4),(5,0),(5,1),(0,7),(1,7),(2,7),(3,7)]
rrt = rrtPath(myGrid,obsCoords,startCoords,endCoords,0.0)
rrt.findPath()
rrt.showBaseGrid()
rrt.showPlots()
"""