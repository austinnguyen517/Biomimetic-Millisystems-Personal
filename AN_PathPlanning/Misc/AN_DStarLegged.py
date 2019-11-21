import vrep
import numpy as np
import matplotlib.pyplot as plt
import sys
import heapq
import math
from AN_DStarAgent import DStarAgent

LSignalName = "CycleLeft"
RSignalName = "CycleRight"

class LeggedDStar(DStarAgent):
    def __init__(self, client, leftName, rightName):
        super(LeggedDStar, self).__init__(client)
        #robot movement
        self.LSignalName = leftName
        self.RSignalName = rightName
        self.CycleFreqL = 1
        self.CycleFreqR = 1

    def goStraight(self):
        self.CycleFreqL = 4
        self.CycleFreqR = 4
        self.sendSignal()

    def turnRight(self):
        self.CycleFreqL = 4
        self.CycleFreqR = 2
        self.sendSignal()

    def turnLeft(self):
        self.CycleFreqL = 2
        self.CycleFreqR = 4
        self.sendSignal()

    def goBack(self):
        self.CycleFreqL = -4
        self.CycleFreqR = -4
        self.sendSignal()

    def stopRobot(self):
        self.CycleFreqL = 0
        self.CycleFreqR = 0
        self.sendSignal()

    def sendSignal(self):
        r = vrep.simxSetFloatSignal(self.clientID, self.LSignalName, self.CycleFreqL, vrep.simx_opmode_oneshot)
        r = vrep.simxSetFloatSignal(self.clientID, self.RSignalName, self.CycleFreqR, vrep.simx_opmode_oneshot)

    def clearSignal(self):
        vrep.simxClearFloatSignal(self.clientID, self.LSignalName, vrep.simx_opmode_oneshot)
        vrep.simxClearFloatSignal(self.clientID, self.RSignalName, vrep.simx_opmode_oneshot)

vrep.simxFinish(-1) #clean up the previous stuff
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection unsuccessful')
    sys.exit('Error: Could not connect to API server')
agent = LeggedDStar(clientID, LSignalName, RSignalName)
agent.prepare()
agent.policy()
