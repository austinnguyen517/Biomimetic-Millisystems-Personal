#! /usr/bin/env python

import numpy as np
import rospy
import vrep
from geometry_msgs.msg import Vector3
from dstar_nav.msg import robotData, cliff
from std_msgs.msg import String
import matplotlib.pyplot as plt
import sys
import heapq
import math
import operator
from dStarEnvironment import Environment
import time


'''Angles returned: [0] positive means angle inclination. Negative indicates angle decline
                    [1] amount of twist. Looking at robot behind to front: Positive means clockwise, negative counter
                    [2] angle from bottom of x axis of world frame '''
'''Proximity Sensor Vector (Normal unit vector to surface detected):
                    [0] positive is left of robot
                    [1] positive is up from robot
                    [2] positive is away from robot'''

'''TODO: Issue: adding an obstacle is not in sync with computeshortestpath -> infinite values are being propagated throug hand used for update start due to 
    overconsistent nodes. Must somehow solve this timing issue and do things sequentially for the algorithm to work.'''

WALLBACK = 150000
SLOPETRAVEL = 2
MINDISTANCE = .02
PADDING = 1 #degrees of padding. Specifies what range of qualifies as "'going in the desired direction'"
RESOLUTION = 8.0
CLIFFTHRESHOLD = .01 #height of a cliff to be considered dangerous
MAPDIMENSIONS = (80, 80, 2)

class DStarAgent(object):
    def __init__(self, dataSub):
        rospy.Subscriber(dataSub, String, self.getRobotData, queue_size = 1)
        #rospy.Subscriber("/data3D", String, self.getData3D, queue_size = 1)
        rospy.Subscriber("/proxyObstacle", Vector3, self.addObstacle, queue_size = 1)
        #rospy.Subscriber("/proxyCliff", cliff, self.addCliff, queue_size  = 100)
        #self.cliffPub = rospy.Publisher("/newCliff", cliff, queue_size = 100)
        self.obstaclePub = rospy.Publisher("/newObstacle", Vector3, queue_size = 1)
        self.env = Environment(RESOLUTION, MAPDIMENSIONS, CLIFFTHRESHOLD)
        self.keyFactor = 0
        self.open = list()
        self.obstacles = set()
        self.obstacleQueue = []
        self.orientation = None
        self.distance = None 
        self.proxVec = None 
        self.data3D = None 

    def policy(self):
        ########## WRAPPER FOR GENERAL AGENT POLICY ##################
        while (self.env.robotPosition != self.env.goalPosition):
            self.manageObstacleQueue()
            ########### CONTINUE UNTIL DETECT NEW OBSTACLE ##############
            while(len(self.obstacleQueue) == 0):
                self.computeShortestPath() #update the map
                ########### CHECK FOR OBSTACLE ##############
                pos = self.env.robotPosition    
                #cliffs = self.checkGround(pos)
                #if pos in self.env.cliffs.keys():
                #    cliffs = cliffs - self.env.cliffs[pos] #difference
                #if len(cliffs) > 0:
                #    print("NEW CLIFF DETECTED")
                #    self.stopRobot()
                #    self.manageCliff(pos, insert)
                #    break
                obstacleAndLocation = self.checkProximity()
                if obstacleAndLocation[0]:
                    self.stopRobot()
                    self.manageObstacle(obstacleAndLocation[1])
                    time.sleep(2) #give time for the proxy to work
                    break
                ########## CHOOSE OPTIMAL POSITION TO TRAVEL TO #############
                neighbors = self.env.neighbors(pos)
                costs = []
                dots = []
                angle = self.orientation
                angle = angle[2]
                v = (math.cos(angle), math.sin(angle))
                for n in neighbors:
                    costs += [self.env.edge(pos, n) + self.env.map[n[0], n[1], 0]]
                    dots += [self.env.dotProduct((n[0] - pos[0], n[1] - pos[1]),v)]

                ############## UPDATE POSITION #############
                i = dots.index(max(dots))
                if costs[i] == np.inf and neighbors[i] not in self.obstacles:
                    #if the direction we are facing has infinite value and is a cliff
                    self.updateStart((0,0), "back")
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
        self.obstacles.add(location)
        send = Vector3()
        send.x = location[0]
        send.y = location[1]
        send.z = 0 #dummy
        self.obstaclePub.publish(send)

    def addObstacle(self, message):
        location = (int(message.x), int(message.y))
        self.obstacleQueue.append(location)

    def manageObstacleQueue(self):
        while len(self.obstacleQueue) > 0:
            location = self.obstacleQueue.pop()
            self.env.map[location[0], location[1], 0] = np.inf
            self.env.map[location[0], location[1], 1] = np.inf
            inQueue = [entry for entry in self.open if entry[1] == location]
            for e in inQueue:
                self.open.remove(e)
            neighbors = self.env.neighbors(location)
            for n in neighbors:
                if n not in self.obstacles:
                    self.updateState(n)


    def manageCliff(self, robPos, cliffs):
        send = cliff()
        send.coordinate = [robPos[0], robPos[1], robPos[2]]
        send.vectors = self.env.tuplesToList(cliffs)
        self.cliffPub.publish(send)

    def addCliff(self, message):
        robPos = message.coordinate 
        cliffs = self.env.listToTuples(message.vectors)
        self.env.cliffs[robPos] = self.env.cliffs[robPos].union(cliffs) if robPos in self.env.cliffs.keys() else cliffs
        update = []
        for vector in cliffs:
            update += [tuple(map(operator.add, robotPosition, vector))]
        for newCliff in update:
            if (self.env.map[newCliff[0], newCliff[1], 0] != np.inf):
                self.updateState(newCliff)

    def updateStart(self, newPosition, default = None): #pass in default to just take a certain action
        while(True):
            ############ GET INITIAL POSITIONS/ANGLES. CHECK IF TOO CLOSE TO OBSTACLE ############
            position = self.env.robotPosition
            angle = self.orientation
            distance = self.distance
            if distance != -1 and distance < MINDISTANCE:
                self.backRobot()
                position = self.env.robotPosition
                break
            else:
                ########## TRAVEL TO THE NEW POSITION ##############
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
            if position != self.env.robotPosition:
                difference = self.env.euclidian(position, self.env.robotPosition) #the difference in heuristic is this anyway
                self.keyFactor += difference
                break

    def updateState(self, s):
        ######## UPDATE THIS STATE BY CALCULATING NEW RHS VALUE ##########
        if s != self.env.goalPosition:
            gPlusEdge = []
            for n in self.env.neighbors(s):
                gPlusEdge += [self.env.map[n[0], n[1], 0] + self.env.edge(s, n)]
            minimum = min(gPlusEdge)
            self.env.map[s[0], s[1], 1] = minimum
        inQueue = [entry for entry in self.open if entry[1] == s]

        ######### REMOVE FROM QUEUE IF PRESENT ##########
        if len(inQueue) > 0:
            self.open.remove(inQueue[0])
        ######## ADD BACK IN WITH UPDATED VALUES IF INCONSISTENT ##########
        if self.env.map[s[0], s[1], 0]  != self.env.map[s[0], s[1], 1]:
            heapq.heappush(self.open, (self.key(s), s))

    def computeShortestPath(self):
        ######## PROPAGATE ALL CHANGES TO FIND SHORTEST PATH ############          
        pos = self.env.robotPosition
        count = 0
        while (len(self.open) > 0 and ((heapq.nlargest(1, self.open)[0][0] < self.key(pos)) or (self.env.map[pos[0], pos[1], 0] != self.env.map[pos[0], pos[1], 1]))):
            node = heapq.heappop(self.open) #the points are already transformed
            currPoint = node[1]
            x = currPoint[0]
            y = currPoint[1]
            g = self.env.map[x, y, 0]
            rhs = self.env.map[x, y, 1]
            neighbors = self.env.neighbors(currPoint)
            if count > 1000: #if we are updating lots of points, stop the robot from moving
                self.stopRobot()
            ######## CHECK FOR CONSISTENCY, UNDERCONSISTENCY, AND OVERCONSISTENCY ##########
            if g == rhs:
                continue
            if g > rhs:
                self.env.map[x,y,0] = rhs
            else:
                self.env.map[x,y,0] = np.inf
                neighbors = neighbors + [currPoint]
            for n in neighbors:
                if n not in self.obstacles:
                    self.updateState(n)
            count += 1

    def checkGround(self, robotPosition):
        table = self.data3D
        if self.dim*self.dim*3 != len(table):
            print("Error with 3D data size")
            return set()
        heights = np.array(table).reshape((self.dim, self.dim, 3))[:,:,0]
        cliffs = self.env.analyzeCliffs(heights) #returns a set of relative locations of cliffs
        return cliffs

    def checkProximity(self):
        ###### DETECT NEW OBSTACLE AND ADD TO OUR ARRAY SPACE GRAPH ############
        if self.distance == -1: #if nothing was detected '''TODO: work on defaults here'''
            return (False, None)
        ##### IF SOMETHING WAS DETECTED ##########
        self.stopRobot()
        distance = self.distance
        vector = self.proxVec
        angle = self.orientation
        currPosition = self.env.robotPositionUT
        vector = self.env.rotate(vector, angle)
        #slope = -((vector[0]**2 + vector[1]**2)**(1/2))/vector[2] if vector[1] > 0 else ((vector[0]**2 + vector[1]**2)**(1/2))/vector[2]
        slope = ((vector[0]**2 + vector[1]**2)**(1/2))/vector[2]
        if abs(slope) < SLOPETRAVEL: #if the detection has a slope considered travelable
            return (False, None)
        xdist = math.cos(angle[2]) * distance
        ydist = math.sin(angle[2]) * distance              
        location = (xdist + currPosition[0], ydist + currPosition[1])
        location = self.env.transform(location)
        ####### IF IT IS A NEW OBSTACLE, RETURN THE RESULTS #########
        if location in self.obstacles:
            return (False, None)
        return (True, location)

    def prepare(self):
        ################ GET ROBOT/GOAL POSITIONS, MAP DIM WITH RESOLUTION, TRANSFORM ###########################
        while (self.env.robotPosition == None or self.env.goalPosition == None):
            x = 1 + 1 #something random
        #while(self.data3D == None):
        #    x = 1+1
        #self.dim = int(math.sqrt(len(self.data3D) / 3))
        ############# INITIALIZE PRIORITY QUEUE ######################
        while(self.orientation == None):
            x = 1 + 1
        self.env.initializeMap()
        heapq.heapify(self.open)
        goalPosition = self.env.goalPosition
        self.env.map[goalPosition[0], goalPosition[1], 1] = 0
        heapq.heappush(self.open, (self.key(goalPosition), goalPosition))

    def backRobot(self):
        for i in range(WALLBACK):
            self.goBack()
        self.stopRobot()

    def key(self, point):
        x = point[0]
        y = point[1]
        cost = min(self.env.map[x,y,0], self.env.map[x,y,1])
        return (cost + self.calcHeuristic(point) + self.keyFactor, cost)

    def calcHeuristic(self, point):
        #calculates the heuristic of a given point. Heuristic should equal the distance from start plus key factor
        return (self.env.euclidian(point, self.env.robotPosition))
    
    def getRobotData(self, message):
        floats = vrep.simxUnpackFloats(message.data)
        self.env.robotPositionUT = (floats[0], floats[1], floats[2])
        self.env.robotPosition = self.env.transform(self.env.robotPositionUT)
        self.env.goalPosition = self.env.transform((floats[3], floats[4], floats[5]))
        self.orientation = (floats[6], floats[7], floats[8])
        self.proxVec = (floats[9], floats[10], floats[11])
        self.distance = floats[12]
    
    #def getData3D(self, message):
     #   self.data3D = vrep.simxUnpackFloats(message.data)

    