import aStar
import time
startTime = time.time()
obsList = [(2,2),(2,3),(2,4),(5,5),(5,6),(6,6),(7,3),(7,4),(7,5),(7,6),(7,8)]
grid = aStar.Grid(10,10,1)
end = (9,8)
start = (1,1)
aStarInst = aStar.aStarPath(grid,obsList,start, end,0.49)
aStarInst.obsRadius = 0; #assume point obstacles...
aStarInst.findPath()
endTime =time.time()#note this is not a perfect test, some processing time spent on plotting...
print("Elapsed time: ",endTime-startTime , " seconds")
print("Cost: ",aStarInst.cost)
aStarInst.showPlots()