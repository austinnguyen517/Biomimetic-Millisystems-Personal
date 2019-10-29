import vrep
import vrepInterface
import numpy as np
import matplotlib.pyplot as plt
import sys
import heapq
import math

LSignalName = "CycleLeft"
RSignalName = "CycleRight"
WALLBACK = 100000
MINDISTANCE = .02
MAXDISTANCE = .25
PADDING = 2 #degrees of padding. Specifies what range of qualifies as "'going in the desired direction'"
RESOLUTION = 8

class FieldDStarAgent:
    def __init__(self, client, leftName, rightName):
        self.clientID = client

        #robot movement
        self.LSignalName = leftName
        self.RSignalName = rightName
        self.CycleFreqL = 1
        self.CycleFreqR = 1
        self.robotHandle = None
        self.robotPosition = None
        self.xOffset = 0
        self.yOffset = 0
        #self.vector = None

        #sensing and planning
        self.proxHandle = None
        self.goalHandle = None
        self.goalPosition = None
        self.mapDimensions = (0,0)
        self.res = RESOLUTION #how many nodes per meter we want to process. Increase resolution to take into account more nodes...but might slow down processing
        self.map = None
        self.keyFactor = 0
        self.open = list()
        self.obstacles = set()

    def policy(self):
        ########## WRAPPER FOR GENERAL AGENT POLICY ##################
        while (not self.goalDone()):
            print("############### RECOMPUTING SHORTEST PATH ###################")
            self.computeShortestPath() #update the map
            print("################### UPDATING START ###########################")
            self.showColorGrid()

            ########### CONTINUE UNTIL DETECT NEW OBSTACLE ##############
            while(True):
                ########### CHECK FOR OBSTACLE ##############
                readings = self.checkProximity()
                newObstacle = readings[0]
                location = readings[1]
                if newObstacle:
                    print("OBSTACLE DETECTED AT ", location)
                    break

                ########## CHOOSE OPTIMAL POSITION TO TRAVEL TO #############
                neighbors = self.neighbors(self.robotPosition)
                costs = []
                for n in neighbors:
                    costs += [self.edge(self.robotPosition, n) + self.map[n[0], n[1], 0]]
                minimum = min(costs)
                indices = [i for i,x in enumerate(costs) if x == minimum]
                candidates = [neighbors[i] for i in indices]

                ######### USE DOT PRODUCT TO PRIORITIZE POINTS IN FRONT OF ROBOT #########
                if len(candidates) == 1:
                    minPoint = candidates[0]
                else:
                    r, angle = vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
                    angle = angle[2]
                    v = (math.cos(self.radToDeg(angle)), math.sin(self.radToDeg(angle)))
                    dots = []
                    for c in candidates:
                        v1 = (c[0] - self.robotPosition[0], c[1] - self.robotPosition[1])
                        dots += [self.dotProduct(v1,v)]
                    minimum = min(dots)
                    minPoint = candidates[dots.index(minimum)]

                ####### UPDATE POSITION ########
                self.updateStart(minPoint)

            ######### DETECTED OBJECT. REMOVE FROM PQ. UPDATE COSTS OF NEIGHBORS ###########
            self.stopRobot()
            obx = location[0]
            oby = location[1]
            inQueue = [entry for entry in self.open if entry[1] == (obx, oby)]
            if len(inQueue) > 0:
                self.open.remove(inQueue[0])
            self.map[obx, oby, 0] = np.inf
            self.map[obx, oby, 1] = np.inf
            neighbors = self.neighbors(location)
            for n in neighbors:
                if n not in self.obstacles:
                    self.updateState(n)

    def updateStart(self, newPosition):
        while(True):
            ############ GET INITIAL POSITIONS/ANGLES. CHECK IF TOO CLOSE TO OBSTACLE ############
            r, position = vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer) #-1 specifies we want the absolute position
            r, angle = vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
            r, distance = vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_buffer)
            if r == vrep.simx_return_ok and distance != -1 and distance < MINDISTANCE:
                print('BACK')
                self.backRobot()
                r, position = vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
                break
            else:
                ########## TRAVEL TO THE NEW POSITION ##############
                position = self.transform(position) #our coordinate frame
                angle = self.radToDeg(angle[2]) #the angles that v-rep gives range from -pi to pi radians. This is hard to work with. Convert to 0 to 360 degrees
                if (angle < 0):
                    angle += 360
                xVec = newPosition[0] - position[0]
                yVec = newPosition[1] - position[1]
                desiredAngle = math.degrees(math.atan(yVec/xVec)) if xVec != 0 else yVec * 90
                if (desiredAngle < 0):
                    desiredAngle += 360
                if desiredAngle - PADDING < angle and desiredAngle + PADDING > angle:
                    self.CycleFreqL = 3
                    self.CycleFreqR = 3
                else:
                    turnRight = ((360 - desiredAngle) + angle) % 360
                    turnLeft = ((360 - angle) + desiredAngle) % 360
                    if turnRight < turnLeft: #turn right if the work to turn right is less than turning left
                        self.CycleFreqL = 3
                        self.CycleFreqR = 1
                    else: #turn left if the work to turn left is less than turning right
                        self.CycleFreqL = 1
                        self.CycleFreqR = 3
                self.sendSignal()

            ######### BREAK AND UPDATE ROBOTPOSITION IF TRANSITIONED ###########
            if position != self.robotPosition:
                self.robotPosition = position
                difference = self.euclidian(position, self.robotPosition) #the difference in heuristic is this anyway
                self.keyFactor += difference
                break


    def updateState(self, s):
        ######## UPDATE THIS STATE BY CALCULATING NEW RHS VALUE ##########
        if s != self.goalPosition:
            minimum = np.inf
            gPlusEdge = []
            for n in self.neighbors(s):
                x = n[0]
                y = n[1]
                gPlusEdge += [self.map[n[0], n[1], 0] + self.edge(s, n)]
            minimum = min(gPlusEdge)
            flag = np.inf in gPlusEdge
            self.map[s[0], s[1], 1] = minimum
        inQueue = [entry for entry in self.open if entry[1] == s]

        ######### REMOVE FROM QUEUE IF PRESENT ##########
        if len(inQueue) > 0:
            self.open.remove(inQueue[0])
        ######## ADD BACK IN WITH UPDATED VALUES IF INCONSISTENT ##########
        if self.map[s[0], s[1], 0] != self.map[s[0], s[1], 1]:
            heapq.heappush(self.open, (self.key(s), s))

    def computeShortestPath(self):
        x = self.robotPosition[0]
        y = self.robotPosition[1]
        ######## PROPAGATE ALL CHANGES TO FIND SHORTEST PATH ############
        while (len(self.open) > 0): #and ((min(self.open)[0] < self.key(self.robotPosition)) or self.map[x,y,0] != self.map[x,y,1])):
            mini = heapq.heappop(self.open) #the points are already transformed
            key = mini[0]
            currPoint = mini[1]
            x = currPoint[0]
            y = currPoint[1]
            g = self.map[currPoint[0], currPoint[1], 0]
            rhs = self.map[currPoint[0], currPoint[1], 1]
            n = self.neighbors(currPoint)

            ######## CHECK FOR CONSISTENCY, UNDERCONSISTENCY, AND OVERCONSISTENCY ##########
            if g == rhs:
                continue
            if g > rhs:
                self.map[x, y, 0] = rhs
            else:
                self.map[x,y,0] = np.inf
                n = n + [currPoint] #add this current point to the list, then decide later if we want to push it back onto the heap
            for neighbor in n:
                if neighbor not in self.obstacles:
                    self.updateState(neighbor)

    def goalDone(self):
        return self.robotPosition == self.goalPosition

    def checkProximity(self):
        ###### DETECT NEW OBSTACLE AND ADD TO OUR ARRAY SPACE GRAPH ############
        r, distance = vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_buffer) #retrieves distance of a detected object from child script
        if r != vrep.simx_return_ok or distance == -1: #if nothing was detected
            return (False, None, np.inf)

        ##### IF SOMETHING WAS DETECTED ##########
        self.stopRobot()
        r, angle = vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
        r, currPosition = vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
        angle = angle[2] #computation is always in radians
        xdist = math.cos(angle) * distance
        ydist = math.sin(angle) * distance
        worldx = xdist + currPosition[0]
        worldy = ydist + currPosition[1]
        location = (worldx, worldy)
        location =self.transform(location)

        ####### IF IT IS A NEW OBSTACLE, RETURN THE RESULTS #########
        if location in self.obstacles:
            return (False, None, np.inf)
        self.obstacles.add(location)
        return (True, location, distance)

    def prepare(self):
        self.clearSignal()

        ################## PREPARE SENSORS ############################
        rCode, self.proxHandle = vrep.simxGetObjectHandle(self.clientID, 'sensor#1', vrep.simx_opmode_oneshot_wait)
        if rCode != vrep.simx_return_ok:
            print("Could not get proximity sensor handle. Exiting.")
            sys.exit()
        error, state, point, handle, vector = vrep.simxReadProximitySensor(self.clientID, self.proxHandle, vrep.simx_opmode_streaming)


        ################ GET ROBOT/GOAL POSITIONS, MAP DIM WITH RESOLUTION, TRANSFORM ###########################
        r, self.robotHandle = vrep.simxGetObjectHandle(self.clientID, 'body#1', vrep.simx_opmode_oneshot_wait)
        r, self.goalHandle = vrep.simxGetObjectHandle(self.clientID, 'Goal', vrep.simx_opmode_oneshot_wait)
        r = -1
        while (r != vrep.simx_return_ok):
            r, self.robotPosition = vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_streaming) #-1 specifies we want the absolute position
        r = -1
        while (r != vrep.simx_return_ok):
            r, self.goalPosition = vrep.simxGetObjectPosition(self.clientID, self.goalHandle, -1, vrep.simx_opmode_streaming)
        rCode = -1
        while (rCode == 1):
            rCode, angle = vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_streaming) #just to prepare our vrep
        self.xOffset = -1 * (min(self.robotPosition[0], self.goalPosition[0]))
        self.yOffset = -1 * (min(self.robotPosition[1], self.goalPosition[1]))
        print("Untransformed robot and goal positions: ", self.robotPosition, self.goalPosition)
        print("X and Y offsets: ", self.xOffset, self.yOffset)
        self.robotPosition = self.transform(self.robotPosition)
        self.goalPosition = self.transform(self.goalPosition)
        print("Transformed robot and goal positions: ", self.robotPosition, self.goalPosition)
        self.mapDimensions = (abs(self.goalPosition[0] - self.robotPosition[0]) + 2, abs(self.goalPosition[1] - self.robotPosition[1]) + 2, 2) #g, rhs
        self.map = np.zeros(self.mapDimensions)
        vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_streaming) #just to get things started


        ############### NAIVELY FILL IN G AND RHS VALUES FOR MAP #########################
        for i in range(self.mapDimensions[0]):
            for j in range(self.mapDimensions[1]):
                cost = self.euclidian(self.goalPosition, (i,j))
                self.map[i,j,0] = cost
                self.map[i,j,1] = cost


        ############# INITIALIZE PRIORITY QUEUE ######################
        print("OVERALL GOAL POSITION: ", self.goalPosition)
        heapq.heapify(self.open)
        self.map[self.goalPosition[0], self.goalPosition[1], 1] = 0
        heapq.heappush(self.open, (self.key(self.goalPosition), self.goalPosition))

    def edge(self, point1, point2): #abstraction for edges
        return self.euclidian(point1, point2) #for now use euclidian distance

    def radToDeg(self, rad):
        #points are returned in radians by orientation
        return (rad/math.pi) * 180

    def transform(self, point):
        #find the place in the array a point (in v-rep coordinates) is
        x = int((point[0] + self.xOffset) * self.res)
        y = int((point[1] + self.yOffset) * self.res)
        return (x,y)

    def clearSignal(self):
        vrep.simxClearFloatSignal(self.clientID, self.LSignalName, vrep.simx_opmode_oneshot)
        vrep.simxClearFloatSignal(self.clientID, self.RSignalName, vrep.simx_opmode_oneshot)

    def stopRobot(self):
        self.CycleFreqL = 0
        self.CycleFreqR = 0
        self.sendSignal()

    def backRobot(self):
        r, distance = vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_buffer)
        while distance < MAXDISTANCE and distance != -1: #while it is not far away enough and there is an obstacle in front of him
            self.CycleFreqL = -2
            self.CycleFreqR = -2
            self.sendSignal()
            r, distance = vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_buffer)
        self.stopRobot()

    def sendSignal(self):
        vrep.simxSetFloatSignal(self.clientID, self.LSignalName, self.CycleFreqL, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(self.clientID, self.RSignalName, self.CycleFreqR, vrep.simx_opmode_oneshot)

    def neighbors(self, point):
        #returns all the neighbors in array coordinates of a given point
        neighbors = []
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                curr = (point[0] + i, point[1] + j)
                if curr[0] >= 0 and curr[1] >= 0 and curr[0] < self.mapDimensions[0] and curr[1]<self.mapDimensions[1] and curr != point:
                    neighbors += [curr]
        return neighbors

    def key(self, point):
        x = point[0]
        y = point[1]
        cost = min(self.map[x,y,0], self.map[x,y,1])
        return (cost + self.calcHeuristic(point) + self.keyFactor, cost)

    def calcHeuristic(self, point):
        #calculates the heuristic of a given point. Heuristic should equal the distance from start plus key factor
        return (self.euclidian(point, self.robotPosition))

    def euclidian(self, point1, point2):
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1])**2) **(1/2)


    def dotProduct(self, v1, v2):
        #given a point, computes the absolute value of dot product of vector: start to point and vector: start to goal
        x1 = v1[0]
        y1 = v1[1]
        norm1 = (x1**2 + y1**2)**(1/2)
        x2 = v2[0]
        y2 = v2[1]
        norm2 = (x2**2 + y2**2)**(1/2)
        #must include normalizations
        return (x1/norm1) * (x2/norm2) + (y1/norm1) * (y2/norm2)

    def showColorGrid(self):
        plt.title('G values: Black is Current Position')
        temp = self.map[self.robotPosition[0],self.robotPosition[1], 0]
        self.map[self.robotPosition[0], self.robotPosition[1], 0] = 0
        heatmap = plt.pcolor(self.map[:,:,0])
        self.map[self.robotPosition[0], self.robotPosition[1], 0] = temp
        plt.show()

    '''UNUSED METHODS'''
    def neighborPairs(self, point):
        #given a specific point on the map, generate a list of the possible neighbor pairs to calculate stuff with
        neighbors = self.neighbors(point)
        res = []
        for i in range(len(neighbors)):
            for j in range(i + 1, len(neighbors)):
                first = neighbors[i]
                second = neighbors[j]
                sum = abs(first[0] - second[0]) + abs(first[1] - second[1]) #checks if these are adjacent neighbors
                if sum == 1:
                    res += [[first, second]]
        #add in first and last as pair if possible
        return res

    def getC(self, point):
        #returns the cost of a cell based on map information. Returns 1 if no map info has been given
        return 1

    def getB(self, point):
        #returns the cost of bottom of cell. Returns 1 if no map info has been given
        return 1

    def adjacent(self, point):
        #returns a list of adjacent points for obstacle buffering
        adjacents = [(point[0] + 1, point[1]), (point[0], point[1] + 1), (point[0] - 1, point[1]), (point[0], point[1] - 1)]
        adjacents = [curr for curr in adjacents if curr[0] >= 0 and curr[1] >= 0 and curr[0] < self.mapDimensions[0] and curr[1]<self.mapDimensions[1]]
        return adjacents

    def manhattan(self, point1, point2):
        return (abs(point1[0] - point2[0]) + abs(point1[1] - point2[1]))

    def inverseTransform(self, point):
        x = point[0] / self.res
        y = point[1] / self.res
        return (x,y)

    def degToRad(self, deg):
        return (deg/180) * math.pi

vrep.simxFinish(-1) #clean up the previous stuff
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection unsuccessful')
    sys.exit('Error: Could not connect to API server')
agent = FieldDStarAgent(clientID, LSignalName, RSignalName)
agent.prepare()
agent.policy()



'''Work with d star lite first...then do field d star'''
'''def computeCost(self, s, s1, s2):
#computes the cost of being in state s given two of the states it can travel to. Minimize over all combinations of s1 and s2 when used
#assumes S2 is the DIAGONAL neighbor of s. s, s1, s2 are tuples representing the points on OUR map coordinate frame
if self.euclidian(s, s2) < self.euclidian(s,s1):
temp = s1
s1 = s2
s2 = s1
cost1 = self.map[s1[0], s1[1]][0]
cost2 = self.map[s2[0], s2[1]][0]
rhs1 = self.map[s1[0], s1[1]][1]
rhs2 = self.map[s2[0], s2[1]][1]
c = self.getC(s)
b = self.getB(s)
if rhs1 == np.inf and rhs2 == np.inf: #checking rhs values
return np.inf
if cost1 <= cost2:
return min(c,b) + cost1 #we assume c and b == 1
f = cost1 - cost2
if f <= b:
if (c <= f):
    return (2 ** (1/2)) * c + cost2
else:
    y = min(f/((c*c + f *f )**(1/2)), 1)
    return c * ((1 + y * y) ** (1/2)) + f *(1-y) + cost2
else:
if (c <= b):
    return c * (2 ** (1/2)) + cost2
else:
    x = 1 - min(b / ((c * c - b * b) **(1/2)), 1)
    return c*((1 + (1-x)**2)**(1/2)) + b*x + cost2'''
