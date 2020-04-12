#! /usr/bin/env python

#Includes general multirobot collaborative agent
#Interacts with arbitrary environment defined outside
#Abstractions include nodes and edges...which may be arbitrarily placed depending on environment definition ie occupancy grid or topological graph
#Contains local map that may be shared within a certain radius with other robots. 
#Greedy algorithm initially defined to test

'''Improvements later on:
- Idea:
    - Make this the multiAgent hive framework
    - Have each robot be a Dstar agent meaning all it needs to do is travel to a specific location!


- Adding in ROS cooperative functionality
- Work with the fact that we have only ONE sensor (not omnidirectional...this means fixing the algorithm to accomodate ie. does it move? or just observe?)

- Work with propagation of utilities based on opportunity...or add a heuristic
- Defining proper probability of detection and PCD for opportunity (probabatility detection should be based on hidden edges?)
- Using topological maps to save space 
- Using the LSR and LRR regions for the nodes
- Shifting to 3-D space for generalization
- Using 360 degree sensors

'''

import numpy as np
import rospy
import vrep
import matplotlib.pyplot as plt
import sys
import heapq
from gridEnv import gridWorld

dimensions = (10,10) #in meters
resolution = 10
goalEntropy = 100
WALLBACK = 10000
SLOPETRAVEL = 10    
ROUNDTHRESH = .95


'''Angles returned: [0] positive means angle inclination. Negative indicates angle decline
                    [1] amount of twist. Looking at robot behind to front: Positive means clockwise, negative counter
                    [2] angle from bottom of x axis of world frame '''
'''Proximity Sensor Vector (Normal unit vector to surface detected):
                    [0] positive is left of robot
                    [1] positive is up from robot
                    [2] positive is away from robot'''

def infoAgent():
    def __init__(self, numRobots, index, dimensions, resolution, histRes, roundThresh):
        self.numAgents = numRobots
        self.agents = [] #list of the robot class instances?? 
        self.robots = {} #dictionary of dictionaries. Each robot corresponds to a different set or parameters, local maps, conditions, and topics
    
        self.gEnv = gridWorld(dimensions, resolution, threshold = ROUNDTHRESH) #global map
        self.hist = np.ones(histRes) #based on the max detection distance of sensor
        self.histSize = histRes
        self.maxSense = None 
        self.frontier = {} #start by making this a global frontier ie. put this in the globalMap 
        #eventually, put this in the local maps. Merge maps in the global and merge frontiers as well?


    def policy(self):
        ########## WRAPPER FOR GENERAL AGENT POLICY ##################
        '''TODO: edit this to be the overall HIVE manager instead of a single robot'''
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
    
    def manageMapData(self):
        #updates for the map data
        pass
    
    def manageRobotData(self):
        #robot positions from agents.py 
        pass 

    def exploreDone(self):
        return self.env.entropy() <= goalEntropy
    
    def updateHistogram(self, observation):
        #check the bin size 
        #observation is a distance. So, increment the bin that corresponds to that distance
        binWidth = self.maxSense/len(self.hist)
        numBins = np.floor(observation / binWidth)
        self.hist[numBins] += 1
        self.histSize += 1

    def probDetect(self, distance):
        #probability of being able to sense a space from a specific spot depends on distance 
        #check the histogram points so far. Find bin corresponding to distance 
        #fraction over size of histogram
        binWidth = self.maxSense/len(self.hist)
        binNumber = np.floor(distance/binWidth)
        numObserved = 0
        for i in range(binNumber, len(self.hist)):
            numObserved += self.hist[i]
        return numObserved / self.histSize

    def computeUtility(self, point):
        #for now, make it greedy 
        #compute the total entropy of the points that we remotely have a good probability of detecting (histogram)
        #compute the expected entropy after observation of all the points IF the new observation is closer than previous ones based on the PCD 
        #use the formula on the whiteboard

        #consider just SAMPLING points from the options to fix the number of points we use to compute the utility!! From what distribution? 
        #does not include utility of OWN position yet
        xRange = (max(0, point[0] - (self.maxSense) * self.env.resolution), min(self.env.mapDim[0], point[0] + (self.maxSense * self.resolution)))
        yRange = (max(0, point[1] - (self.maxSense) * self.env.resolution), min(self.env.mapDim[1], point[1] + (self.maxSense * self.resolution)))
        utility = 0
        for i in range(xRange[0], xRange[1]):
            for j in range(yRange[0], yRange[1]):
                distance = self.env.euclidian(point, (i,j))
                PCD = self.PCD(distance)
                currProb = self.env.getProb(i,j)
                if abs(PCD - .5) > abs(currProb - .5):
                    currEnt = -currProb*np.log(currProb) - (1-currProb)*np.log(1-currProb)
                    expectEnt = -(currProb)*np.log(PCD) - (1-currProb)*np.log(1-PCD)
                    utility += PCD * (currEnt - expectEnt)
        return utility

    def discountUtility(self, point):
        #given a point, discount all the utilities of surrounding areas based on the histogram bins
        #multiply all utility points by P(won't detect)

        #for now, assume we have a fixed frontier! Iterate through the list of frontier nodes instead of all nodes in utility array 
        for item in self.frontier:
            distance = self.env.euclidian(point, item)
            if distance < self.maxSense:
                newU = (1 - self.probDetect(distance)) * self.env.getUtility(item[0], item[1])
                self.setUtility(item[0], item[1], newU)
        
