#! /usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from dstar_nav.msg import robotData
import matplotlib.pyplot as plt
import sys
import heapq
import math
import operator


'''Angles returned: [0] positive means angle inclination. Negative indicates angle decline
                    [1] amount of twist. Looking at robot behind to front: Positive means clockwise, negative counter
                    [2] angle from bottom of x axis of world frame '''
'''Proximity Sensor Vector (Normal unit vector to surface detected):
                    [0] positive is left of robot
                    [1] positive is up from robot
                    [2] positive is away from robot'''

WALLBACK = 150000
SLOPETRAVEL = 1
MINDISTANCE = .02
MAXDISTANCE = .25
PADDING = 1 #degrees of padding. Specifies what range of qualifies as "'going in the desired direction'"
RESOLUTION = 8
MAPDIMENSIONS = (100, 100 ,4) #g, rhs, slopes, elevation
CLIFFTHRESHOLD = .01 #height of a cliff to be considered dangerous

class DStarAgent:
    def __init__(self, client):
        self.clientID = client
        self.robotHandle = None
        self.proxHandles = [] #[0] front-facing proximity sensor
        self.goalHandle = None
        self.keyFactor = 0
        self.open = list()
        self.env = Environment(RESOLUTION, MAPDIMENSIONS, CLIFFTHRESHOLD)
        self.currHeight = 0
        self.theta = None #this represents the angle of depression of our ground proximity sensor
        self.z = None #this represents the height of the ground proximity sensor
        self.noCliff = None #this represents the maximum value for a distance to not be considered a cliff

    def policy(self):
        ########## WRAPPER FOR GENERAL AGENT POLICY ##################
        while (not self.env.goalDone()):
            print("RECOMPUTING SHORTEST PATH")
            self.computeShortestPath() #update the map
            self.env.showColorGrid()
            print("UPDATING START")
            ########### CONTINUE UNTIL DETECT NEW OBSTACLE ##############
            while(True):
                ########### CHECK FOR OBSTACLE ##############
                pos = self.env.getRobotPosition()
                cliffs = self.checkGround(pos)
                if pos in self.env.cliffs.keys():
                    cliffs = cliffs - self.env.cliffs[pos] #difference
                if len(cliffs) > 0:
                    print("NEW CLIFF DETECTED")
                    self.stopRobot()
                    self.manageCliff(pos, cliffs)
                    break
                obstacleAndLocation = self.checkProximity()
                if obstacleAndLocation[0]:
                    print("NEW OBSTACLE DETECTED")
                    self.stopRobot()
                    self.manageObstacle(obstacleAndLocation[1])
                    break
                ########## CHOOSE OPTIMAL POSITION TO TRAVEL TO #############
                neighbors = self.env.neighbors(pos)
                costs = []
                dots = []
                #CHANGE
                r,angle = vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
                robotPosition = self.env.getRobotPosition()
                angle = angle[2]
                v = (math.cos(angle), math.sin(angle))
                for n in neighbors:
                    costs += [self.env.edge(pos, n) + self.env.getMap(n[0], n[1], 0)]
                    dots += [self.env.dotProduct((n[0] - robotPosition[0], n[1] - robotPosition[1]),v)]

                ############## UPDATE POSITION #############
                if costs[dots.index(max(dots))] == np.inf and neighbors[dots.index(max(dots))] not in self.env.obstacles:
                    #if the direction we are facing has infinite value and is a cliff
                    self.updateStart((0,0), "back")
                #END CHANGE
                else:
                    minimum = min(costs)
                    indices = [i for i,x in enumerate(costs) if x == minimum]
                    if len(indices) == 0:
                        self.updateStart(neighbors[indices[0]])
                    else:
                        dots = [dots[i] for i in indices]
                        candidates = [neighbors[i] for i in indices]
                        minPoint = candidates[dots.index(max(dots))]
                        self.updateStart(minPoint)

    def manageObstacle(self, location):
        ######### DETECTED OBJECT. REMOVE FROM PQ. UPDATE COSTS OF NEIGHBORS ###########
        buffedObstacles = self.env.neighbors(location) + [location]
        for b in buffedObstacles:
            self.env.obstacles.add(b)
            self.env.setMap(b[0], b[1], 0, np.inf)
            self.env.setMap(b[0], b[1], 1, np.inf)
        inQueue = [entry for entry in self.open if entry[1] in buffedObstacles]
        for e in inQueue:
            self.open.remove(e)
        neighbors = []
        for ob in buffedObstacles:
            surrounding = self.env.neighbors(ob)
            neighbors += [n for n in surrounding if n not in self.env.obstacles]
        for n in neighbors:
            self.updateState(n)

    def manageCliff(self, robotPosition, cliffs):
        self.env.cliffs[robotPosition] = self.env.cliffs[robotPosition].union(cliffs) if robotPosition in self.env.cliffs else cliffs #update the set of cliffs
        update = []
        for vector in cliffs:
            update += [tuple(map(operator.add, robotPosition, vector))]
        for newCliff in update:
            self.updateState(newCliff)

    def updateStart(self, newPosition, default = None): #pass in default to just take a certain action
        while(True):
            ############ GET INITIAL POSITIONS/ANGLES. CHECK IF TOO CLOSE TO OBSTACLE ############
            r, position = vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer) #-1 specifies we want the absolute position
            r, angle = vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
            r, distance = vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_buffer)
            if r == vrep.simx_return_ok and distance != -1 and distance < MINDISTANCE:
                self.backRobot()
                r, position = vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
                break
            else:
                ########## TRAVEL TO THE NEW POSITION ##############
                height = position[2]
                position = self.env.transform(position) #our coordinate frame
                if not default:
                    angle = self.env.radToDeg(angle[2]) #the angles that v-rep gives range from -pi to pi radians. This is hard to work with. Convert to 0 to 360 degrees
                    angle = angle + 360 if angle < 0 else angle
                    xVec = newPosition[0] - position[0]
                    yVec = newPosition[1] - position[1]
                    desiredAngle = math.degrees(math.atan(yVec/xVec)) if xVec != 0 else yVec * 90
                    desiredAngle = desiredAngle + 360 if desiredAngle < 0 else desiredAngle

                    if desiredAngle - PADDING < angle and desiredAngle + PADDING > angle:
                        self.goStraight()
                    else:
                        turnRight = ((360 - desiredAngle) + angle) % 360
                        turnLeft = ((360 - angle) + desiredAngle) % 360
                        if turnRight < turnLeft: #turn right if the work to turn right is less than turning left
                            self.turnRight()
                        else: #turn left if the work to turn left is less than turning right
                            self.turnLeft()
                    self.sendSignal()
                else:
                    if default == "back":
                        self.backRobot()
                    else:
                        print("Error: not implemented")
                        sys.exit()

            ######### BREAK AND UPDATE ROBOTPOSITION IF TRANSITIONED ###########
            if position != self.env.getRobotPosition():
                self.currHeight = height
                self.env.updateHeight(position[0], position[1], [0,0,0], self.currHeight)
                self.env.updateRobotPosition(position)
                difference = self.env.euclidian(position, self.env.getRobotPosition()) #the difference in heuristic is this anyway
                self.keyFactor += difference
                break

    def updateState(self, s):
        ######## UPDATE THIS STATE BY CALCULATING NEW RHS VALUE ##########
        if s != self.env.getGoal():
            minimum = np.inf
            gPlusEdge = []
            for n in self.env.neighbors(s):
                x = n[0]
                y = n[1]
                gPlusEdge += [self.env.getMap(n[0], n[1], 0) + self.env.edge(s, n)]
            minimum = min(gPlusEdge)
            flag = np.inf in gPlusEdge
            self.env.setMap(s[0], s[1], 1, minimum)
        inQueue = [entry for entry in self.open if entry[1] == s]

        ######### REMOVE FROM QUEUE IF PRESENT ##########
        if len(inQueue) > 0:
            self.open.remove(inQueue[0])
        ######## ADD BACK IN WITH UPDATED VALUES IF INCONSISTENT ##########
        if self.env.getMap(s[0], s[1], 0) != self.env.getMap(s[0], s[1], 1):
            heapq.heappush(self.open, (self.key(s), s))

    def computeShortestPath(self):
        ######## PROPAGATE ALL CHANGES TO FIND SHORTEST PATH ############
        while (len(self.open) > 0):
            mini = heapq.heappop(self.open) #the points are already transformed
            key = mini[0]
            currPoint = mini[1]
            x = currPoint[0]
            y = currPoint[1]
            g = self.env.getMap(currPoint[0], currPoint[1], 0)
            rhs = self.env.getMap(currPoint[0], currPoint[1], 1)
            neighbors = self.env.neighbors(currPoint)

            ######## CHECK FOR CONSISTENCY, UNDERCONSISTENCY, AND OVERCONSISTENCY ##########
            if g == rhs:
                continue
            if g > rhs:
                self.env.map[x,y,0] = rhs 
            else:
                self.env.map[x,y,0] = np.inf 
                n = n + [currPoint] #add this current point to the list, then decide later if we want to push it back onto the heap
            for n in neighbors:
                if self.env.map[n[0],n[1], 0] != np.inf:
                    self.updateState(n)

    def checkGround(self, robotPosition):
        table = self.data3D
        if self.dim*self.dim*3 != len(table):
            print("Error with 3D data size")
            return set()
        heights = np.array(table).reshape((self.dim, self.dim, 3))[:,:,0]
        cliffs = self.env.analyzeCliffs(heights, CLIFFTHRESHOLD) #returns a set of relative locations of cliffs
        return cliffs

    def checkProximity(self):
        ###### DETECT NEW OBSTACLE AND ADD TO OUR ARRAY SPACE GRAPH ############
        ###### DETECT NEW OBSTACLE AND ADD TO OUR ARRAY SPACE GRAPH ############
        if self.distance == -1: #if nothing was detected '''TODO: work on defaults here'''
            return (False, None)
        ##### IF SOMETHING WAS DETECTED ##########
        self.stopRobot()
        distance = self.distance
        vector = self.proxVec
        angle = self.orientation
        currPosition = self.env.inverseTransform(self.robotPosition, RESOLUTION)
        vector = self.env.rotate(vector, angle)
        slope = -((vector[0]**2 + vector[1]**2)**(1/2))/vector[2] if vector[1] > 0 else ((vector[0]**2 + vector[1]**2)**(1/2))/vector[2]

        if slope < SLOPETRAVEL: #if the detection has a slope considered travelable
            return (False, None)

        xdist = math.cos(angle[2]) * distance
        ydist = math.sin(angle[2]) * distance
        location = self.env.transform((xdist + currPosition[0], ydist + currPosition[1]), RESOLUTION)

        ####### IF IT IS A NEW OBSTACLE, RETURN THE RESULTS #########
        if self.map[location[0], location[1], 0] == np.inf:
            return (False, None)
        return (True, location)

    def prepare(self):

        ################ GET ROBOT/GOAL POSITIONS, MAP DIM WITH RESOLUTION, TRANSFORM ###########################
        while (self.robotPosition == None):
            x = 1 + 1 #something random
        while (self.map == None):
            x = 1 + 1
        self.dim = int((len(self.data3D) / 3) ** (1/2))

        ############# INITIALIZE PRIORITY QUEUE ######################
        goalPosition = self.goalPosition
        self.env.map[goalPosition[0], goalPosition[1], 1] = 0
        robotPosition = self.env.transform(robotPosition)
        goalPosition = self.env.transform(goalPosition)
        self.env.updateRobotPosition(robotPosition)
        self.env.updateGoal(goalPosition)
        self.env.initializeMap()

        ############# INITIALIZE PRIORITY QUEUE ######################
        print("OVERALL GOAL POSITION: ", self.env.getGoal())
        heapq.heapify(self.open)
        heapq.heappush(self.open, (self.key(goalPosition), goalPosition))

    def backRobot(self):
        #r, distance = vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_buffer)
        #while distance < MAXDISTANCE and distance != -1: #while it is not far away enough and there is an obstacle in front of him
        for i in range(WALLBACK):
            self.goBack()
            #r, distance = vrep.simxGetFloatSignal(self.clientID, "ProxDistance", vrep.simx_opmode_buffer)
        self.stopRobot()

    def key(self, point):
        x = point[0]
        y = point[1]
        cost = min(self.env.getMap(x,y,0), self.env.getMap(x,y,1))
        return (cost + self.calcHeuristic(point) + self.keyFactor, cost)

    def calcHeuristic(self, point):
        #calculates the heuristic of a given point. Heuristic should equal the distance from start plus key factor
        return (self.env.euclidian(point, self.env.getRobotPosition()))
