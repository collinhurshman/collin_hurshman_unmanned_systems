import aStar
import time
from numpy import genfromtxt
startTime = time.time()
obsPath = r"D:\collin_hurshman_unmanned_systems\collin_hurshman_unmanned_systems\week_3\biggrids.csv"
obstacles = genfromtxt(obsPath, delimiter=',')
obsList = []
for obs in obstacles:#putting obstacles in the appropriate format...
    obsList.append((obs[0],obs[1]))
#obstacles should now be loaded...

#setting up the grid...
grid = aStar.Grid(50,50,0.5)
end = (49,0.5)
start = (0.5,49)
aStarInst = aStar.aStarPath(grid,obsList,start, end,0.49)
aStarInst.obsRadius = 0; #assume point obstacles...
aStarInst.findPath()
print(aStarInst.cost)
endTime =time.time()#note this is not a perfect test, some processing time spent on plotting...
print("Elapsed time: ",endTime-startTime , " seconds")
print("Cost: ",aStarInst.cost)
aStarInst.showPlots()
