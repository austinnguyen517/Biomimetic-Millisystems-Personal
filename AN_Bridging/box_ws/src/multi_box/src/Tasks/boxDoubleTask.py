#! /usr/bin/env python

#! /usr/bin/env python

from task import Task 
from task import Task, unitVector, dot, vector
from task import distance as dist
import numpy as np 
import torch 
import torch.nn as nn
import vrep
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from boxTask import BoxTask
from collections import namedtuple

Info = namedtuple('Info',('prevPos', 'pos', 'blockPos', 'prevBlock', 'ori', 'prevOri', 'blockOri'))

class BoxDoubleTask(BoxTask):
    def __init__(self):
        super(BoxDoubleTask, self).__init__()
        self.s_n = 12 #TODO: check this dimension
        self.actionMap = {0: (-3,-1), 1:(-1,-3), 2:(-3,-3), 3:(1,3), 4:(3,3), 5:(3,1), 6: (-2, 2), 7: (2, -2)} 
        self.discrete = True


    def extractInfo(self):
        self.vTrain = self.agent.vTrain
        self.pubs = self.agent.pubs
        self.trainMode = self.agent.trainMode
        self.w_phase1 = self.vTrain['w_phase1']
        self.w_phase2 = self.vTrain['w_phase2']
        self.w_phase3 = self.vTrain['w_phase3']
        self.agents = self.agent.agents
        self.name = self.agent.name
        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 
 
    def sendAction(self, s, w_s):
        #pass in the local state of agent and its name according to self.agents
        msg = Vector3()
        ret = self.agent.get_action(s, w_s)
        if self.discrete:
            action = [self.actionMap[r] for r in ret]
        else:
            action = ret
        for (i, key) in enumerate(self.pubs.keys()):
            msg.x, msg.y = (action[i][0], action[i][1])
            self.pubs[key].publish(msg)
        return ret
    
    def rewardFunction(self, s_n, a):
        first, second = self.splitState(s_n.ravel().tolist())
        fPrev, sPrev  = self.splitState(self.prev['S'].ravel().tolist())
        
        prevPos, pos, blockPos, prevBlock, ori, prevOri, blockOri = self.unpack(fPrev, first, double=True)
        first = Info(prevPos, pos, blockPos, prevBlock, ori, prevOri, blockOri)

        prevPos, pos, blockPos, prevBlock, ori, prevOri, blockOri = self.unpack(sPrev, second, double=True)
        second = Info(prevPos, pos, blockPos, prevBlock, ori, prevOri, blockOri)

        if self.phase == 1:
            if first.pos[2] < .35 or second.pos[2] < .35:
                return (-3, 1)
            if blockPos[-1] < .3:
                self.phase += 1
                return (5, 0)
            box_r = (blockPos[0] - prevBlock[0]) - .005*(abs(blockOri))  

            vel_r = dist(first.prevPos, prevBlock) - dist(first.pos, blockPos)
            vel_r += dist(second.prevPos, prevBlock) - dist(second.pos, blockPos)

            prevVec = unitVector(vector(first.prevOri))
            vec = unitVector(vector(first.ori))
            goal = unitVector(blockPos[:2]-first.pos[:2])
            prevDot = dot(prevVec, goal)
            currDot = dot(vec, goal)
            ori_r = currDot - prevDot

            prevVec = unitVector(vector(second.prevOri))
            vec = unitVector(vector(second.ori))
            goal = unitVector(blockPos[:2]-second.pos[:2])
            prevDot = dot(prevVec, goal)
            currDot = dot(vec, goal)
            ori_r += currDot - prevDot

            r = (25*box_r + vel_r + 2*ori_r) - .01
        if self.phase == 2:
            if first.pos[2] < .35 or second.pos[2] < .35:
                return (-6, 1)
            if first.pos[0] > .45 and second.pos[0] > .45:
                print('Success!')
                return (5, 1)
            
            vel_r = first.pos[0] - first.prevPos[0] 
            vel_r += second.pos[0] - second.prevPos[0]

            y_r = .1* (abs(first.pos[1] - blockPos[1]) + abs(second.pos[1] - blockPos[1]))

            #ori_r = .05* (abs(first.ori) + abs(second.ori))

            r = vel_r - y_r - .01
        return (r, 0)

    
    def splitState(self, s):
        box = np.round(np.array(s[4:8]), decimals = 2)
        first = np.round(np.array(s[:8]), decimals = 2)
        second = np.round(np.array(s[8:12] + s[4:8]), decimals=2)
        
        #append relative information to include observations for each local state
        first = np.hstack((first, second[:2] - first[:2], second[3:4]))
        second = np.hstack((second, first[:2] - second[:2], first[3:4]))

        return first, second


    def receiveState(self, msg):
        floats = vrep.simxUnpackFloats(msg.data)
        self.goal = np.array(floats[-4:-1])
        fail = floats[-1]
        restart = 0
        floats = floats[:self.s_n]
        first, second = self.splitState(floats)
        
        s = (np.array(floats)).reshape(1,-1)
        first = torch.FloatTensor(first).view(1,-1)
        second = torch.FloatTensor(second).view(1,-1)
        a = (self.sendAction(s, [first, second]))
        if type(self.prev["S"]) == np.ndarray:
            r, restart = self.rewardFunction(s, self.prev['A']) 
            self.agent.store(self.prev['S'], self.prev["A"], r, s, a, restart, self.prevLocals, (first, second)) 
            self.currReward += r
        self.prev["S"] = s
        self.prev["A"] = a
        self.prevLocals = (first, second)
        s = s.ravel()
        if self.trainMode:
            self.agent.train()
        self.restartProtocol(restart or fail)
        return 