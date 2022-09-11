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
        

class aStarPath():
    def __init__(self,grid:Grid,obsList:list,start:tuple,goal:tuple,robotRadius:float):
        self.grid = grid
        self.obsList = obsList
        self.start = start
        self.end = goal
        self.radius = robotRadius
        self.obsRadius = self.grid.spacing/2 
        self.path = []
        self.cost = 0
    
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
        unvisited = {}
        visited = {} 

        currentNode = Node(self.start[0],self.start[1],0,-1)
        currentNode.estCost = self.distance(self.start,self.end)
        currentIndex = self.getNode(self.start[0],self.start[1],self.grid.Xmax,self.grid.spacing)
        unvisited[currentIndex] = currentNode
        if plotDesired: #only set up a figure if the user requests it
            plt.figure()
            plt.title("Node Obstacles with Best Path Highlighted")
            plt.axis([0,self.grid.Xmax+ self.grid.spacing,0,self.grid.Ymax+self.grid.spacing])
        

        while len(unvisited):#iterate as long as there are unvisited nodes...
            #note that in some cases, it may be better to stop as soon as the end is reached...
            currentIndex = min(unvisited,key=lambda x:unvisited[x].estCost)
            currentNode = unvisited[currentIndex]
            visited[currentIndex] = unvisited[currentIndex]
            del unvisited[currentIndex]#remove the current node from unvisited...
            #only plot each node after lowest cost is known, ie the node is visited

            """Below can be uncommented if you want to see the true costs of the nodes visited
            if plotDesired:
                if self.__checkValidity(currentNode):
                    plt.text(currentNode.X,currentNode.Y,str('%.2f' % float(currentNode.cost)),color = "green",fontsize = 6)#only show a couple decimal places in cost
                else:#show obstacles as black
                    plt.text(currentNode.X,currentNode.Y,str('%.2f' % float(currentNode.cost)),color = "red",fontsize = 6)#only show a couple decimal places in cost
            """

            if currentNode.X == self.end[0] and currentNode.Y == self.end[1]:
                break#BREAK ONLY AFTER THE END NODE HAS BECOME VISITED...

            #evaluate the neighbors...
            for i in [-self.grid.spacing, 0, self.grid.spacing]:
                for j in [-self.grid.spacing,0,self.grid.spacing]:        
                    tempX = currentNode.X + i
                    tempY = currentNode.Y + j
                    tempNode = Node(tempX,tempY,0,currentIndex)#use current node index as parent index...
                    tempIndex = self.getNode(tempNode.X,tempNode.Y,self.grid.Xmax,self.grid.spacing)
                    costCalc = self.distance([currentNode.X,currentNode.Y],[tempNode.X,tempNode.Y])
                    tempNode.estCost = currentNode.cost + costCalc + self.distance(self.end,(tempNode.X,tempNode.Y))
                    tempNode.cost = currentNode.cost + costCalc #retain the actual cost...
                    
                    #notice including self.distance(...) is the distinguishing trait of aStar....
                    #it looks ahead at the approx distance to the end goal and only follows paths that have low costs with this distance included,
                    #only update dictionaries if the node provides a new, viable path        
                    if self.__checkValidity(tempNode) and tempIndex not in visited: #make sure the location is valid and not hte 
                        if tempIndex in unvisited:#already looked at this node... is there a shorter path to it?
                            if tempNode.cost < unvisited[tempIndex].cost:
                                tempNode.parent_index = currentIndex
                        else:
                            tempNode.parent_index = currentIndex
                            unvisited[tempIndex] = tempNode #update unvisited if we discover a new unvisited node...
                            
        #start at end node and work backwards...
        endIndex = self.getNode(self.end[0],self.end[1],self.grid.Xmax,self.grid.spacing)    
        self.cost = currentNode.cost#get cost at end node...
        parent = visited[endIndex].parent_index#iterate backwards through the list
        self.path.append(self.end)
        while parent != -1:
            temp = visited[parent]#go up to the next closest node
            parent = temp.parent_index
            self.path.append((temp.X,temp.Y))

        self.path = self.path[::-1]#reverse the path order so it travels start to finish
        if plotDesired:
            for i in range(0, len(self.path)-1):
                point1 = self.path[i]
                point2 = self.path[i+1]
                xvals = [point1[0], point2[0]]
                yvals = [point1[1], point2[1]]
                plt.plot(xvals,yvals, 'b.-')
                #NOW NEED TO ITERATE FROM ONE POINT TO THE NEXT, DRAWING LINE...
            for obs in self.obsList:
                plt.plot(obs[0],obs[1],marker = ".",color = "r")
            for idx in visited:
                node = visited[idx]
                plt.plot(node.X,node.Y,marker = ".",color = "g")
        return self.path


"""
#below is purely for testing...
myGrid = Grid(10,10,0.5)
startCoords = (0,0)
endCoords = (8,9)
obsCoords = [(1,1),(4,4),(3,4),(5,0),(5,1),(0,7),(1,7),(2,7),(3,7)]
aStar = aStarPath(myGrid,obsCoords,startCoords,endCoords,0.0)
aStar.findPath()
aStar.showBaseGrid()
aStar.showPlots()
"""