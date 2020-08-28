#! /usr/bin/env python

import numpy as np
class Task(object):
    def __init__(self):
        self.init = False
        
    def initAgent(self, agent):
        self.agent = agent 
        self.agents = agent.agents
        self.extractInfo()
        self.init = True  

    def extractInfo(self):
        pass

    def sendAction(self,s):
        pass
    
    def rewardFunction(self, s, a):
        pass

    def receiveState(self, msg):
        pass
    
    def restartProtocol(self, restart):
        pass

    def postTraining(self):
        pass

def vector(ori):
    return np.array([np.cos(ori), np.sin(ori)])

def distance(point1, point2):
    assert point1.size == point2.size
    squareSum = sum([(point1[i] - point2[i])**2 for i in range(point1.size)])
    return np.sqrt(squareSum)

def unitVector(v):
    norm = distance(v, np.zeros(v.shape))
    return v/norm

def dot(v, u):
    return np.sum(np.multiply(v, u))