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

#This agent has functionality to:
    #Move to a specific space in the map 
    #Update its local map based on observations it makes
    #Pass that information to the global map to be updated


'''Angles returned: [0] positive means angle inclination. Negative indicates angle decline
                    [1] amount of twist. Looking at robot behind to front: Positive means clockwise, negative counter
                    [2] angle from bottom of x axis of world frame '''
'''Proximity Sensor Vector (Normal unit vector to surface detected):
                    [0] positive is left of robot
                    [1] positive is up from robot
                    [2] positive is away from robot'''

WALLBACK = 150000
SLOPETRAVEL = 2
MINDISTANCE = .02
PADDING = 1 #degrees of padding. Specifies what range of qualifies as "'going in the desired direction'"
RESOLUTION = 8.0
CLIFFTHRESHOLD = .01 #height of a cliff to be considered dangerous
MAPDIMENSIONS = (80, 80, 2)

#PCD Parameters
ALPHA = 1/4
BETA = 10 

class DStarAgent(object):
    def __init__(self, ID):
        self.ID = ID
        self.keyFactor = 0
        self.open = list()
        self.orientation = None
        self.distance = None 
        self.proxVec = None 
        self.data3D = None 
    
    def moveShortest(self, currPosition, goalPosition, map):
        #compute the shortest path
        #execute the shortest path
        #return
        

    def policy(self):
        while (!self.exploreDone()):
            ##### CHECK FOR OBSTACLES. UPDATE MAP ACCORDINGLY
            obstacle, location = self.checkProximity() 
            self.manageObservation(location, obstacle)

            ########### COMPUTE UTILITIES OF FRONTIER ##############
            pos = self.env.robotPosition  
            newFrontier = self.env.frontier(pos[0], pos[1])
            utilities = []
            dots = []
            angle = self.orientation[2]
            v = (math.cos(angle), math.sin(angle))
            for n in newFrontier:
                if self.env.getProb(n[0], n[1]) != 0 and self.env.getProb(n[0], n[1]) != 1:
                    u = self.computeUtility(n) - self.env.euclidian(pos, n)
                    utilities.append(u)
                    self.frontier.add(n)
                    self.env.setUtility(n[0],n[1],u)
                    dots.append(self.env.dotProduct((n[0] - pos[0], n[1] - pos[1]),v))

            ############## UPDATE POSITION #############
            maximum = max(utilities)
            indices = [i for i,x in enumerate(costs) if x == maximum]
            dots = [dots[i] for i in indices]
            candidates = [neighbors[i] for i in indices]
            maxPoint = candidates[dots.index(max(dots))]
            self.frontier.remove(maxPoint)
            self.updatePosition(maxPoint)
            
    def manageObservation(self, location, obstacle):
        #change entropy on the map of location based on PCD and distance
        #passed in are the location of the observation, distance of the observation, and whether an obstacle was observed

        #TODO: support observations along the line of sight passed in. This should be updating the LOCAL map
        #basically, make a loop that computes PCD of each cell along the line of sight and does the exact thing 

        
        probability = self.PCD(distance)
        if obstacle == False:
            probability = 1 - probability 
        prevCertainty = abs(.5 - (self.env.getProb(location[0], location[1])))
        certainty = abs(.5 - probability)
        if certainty > prevUncertainty:
            self.env.setProb(location[0], location[1], probability)

    def updateStart(self, newPosition, default = None): #pass in default to just take a certain action
        #TODO: CHANGE THIS TO A DIJKSTRA'S ALGORITHM IMPLEMENTATION! REQUIRES SHORTEST PATH ALGORITHM OF SOME SORT!

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
                break

    def computeShortestPath(self):
        ######## PROPAGATE ALL CHANGES TO FIND SHORTEST PATH ############          
        pos = self.env.robotPosition
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

    def PCD(self, distance):
        #define a function with tuneable parameters that gives the probabiity of correct detection given distance 
        #consider neural net for this
        return 1/(1+np.exp(-(ALPHA*distance - BETA)))

    def checkProximity(self):
        ###### DETECT NEW OBSTACLE AND ADD TO OUR ARRAY SPACE GRAPH ############
        if self.distance == -1: #if nothing was detected '''TODO: work on defaults here'''
            distance = self.maxSense 
        else:
            distance = self.distance
        ##### IF SOMETHING WAS DETECTED ##########
        self.stopRobot()
        distance = self.distance
        vector = self.proxVec
        angle = self.orientation
        currPosition = self.env.robotPositionUT
        vector = self.env.rotate(vector, angle)
        slope = ((vector[0]**2 + vector[1]**2)**(1/2))/vector[2]

        xdist = math.cos(angle[2]) * distance
        ydist = math.sin(angle[2]) * distance              
        location = (xdist + currPosition[0], ydist + currPosition[1])
        location = self.env.transform(location)

        if abs(slope) < SLOPETRAVEL: #if the detection has a slope considered travelable
            return False, location
        return True, location

    def backRobot(self):
        for i in range(WALLBACK):
            self.goBack()
        self.stopRobot()

    