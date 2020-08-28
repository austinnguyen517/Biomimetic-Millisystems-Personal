#! /usr/bin/env python

from task import Task, unitVector, dot, vector
from task import distance as dist
import numpy as np 
import rospy
import torch 
import torch.nn as nn
import vrep
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt

class BoxTask(Task):
    def __init__(self):
        super(BoxTask, self).__init__()
        self.actionMap = {0: (-2,-1), 1:(-1,-2), 2:(-2,-2), 3:(1,2), 4:(2,2), 5:(2,1), 6: (-2, 2), 7: (2, -2)} 
        self.discrete = True
        self.prev = {"S": None, "A": None}
        self.s_n = 12
        self.currReward = 0
        self.rewards = []
        self.phase = 1

    def extractInfo(self):
        self.vTrain = self.agent.vTrain
        self.pubs = self.agent.pubs
        self.trainMode = self.agent.trainMode
        self.name = self.agent.name
        self.w_phase1 = self.vTrain['w_phase1']
        self.w_phase2 = self.vTrain['w_phase2']
        self.w_phase3 = self.vTrain['w_phase3']
        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 
 
    def sendAction(self, s):
        msg = Vector3()
        ret = self.agent.get_action(s)
        if self.discrete:
            action = self.actionMap[ret]
        else:
            action = ret
        msg.x, msg.y = (action[0], action[1])
        self.pubs[self.name].publish(msg)
        return ret

    def checkPhase(self, pos, blockPos, ori, blockOri, phase):
        if pos[-1] < .35:
            return (-3, 1)
        if phase == 1:
            if blockPos[2] < .3:
                return (5, 0)
        if phase == 2:
            if pos[0] > .75:
                return (5, 1)
        return (0,0)
    
    
    def getAux(self, pos, prevPos, blockPos, prevBlock, ori, prevOri, phase):
        if phase == 1:
            dist_r = (dist(prevPos, blockPos) - dist(pos, blockPos))

            block_r = blockPos[0] - prevBlock[0]
            prevVec = unitVector(vector(prevOri))
            vec = unitVector(vector(ori))
            goal = unitVector(blockPos[:2]-pos[:2])

            prevDot = dot(prevVec, goal)
            currDot = dot(vec, goal)
            ori_r = currDot - prevDot

            return ((block_r + dist_r + 2*ori_r) * self.w_phase1 - .01, 0)

        if phase == 2:
            goal = np.array([.80, blockPos[1], pos[2]])
            delta = dist(pos, goal)
            prevDelta = dist(prevPos, goal)
            dist_r = prevDelta - delta
            y_r = -abs(blockPos[1]-pos[1])

            return ((dist_r  + .15*y_r - .05 * abs(ori)) * self.w_phase3 - .01, 0)

    def unpack(self, prevS, s, double = False):
        if double:
            prevPos = np.array(prevS[:3])
            pos = np.array(s[:3])
            blockPos = np.array(s[4:7])
            prevBlock = np.array(prevS[4:7])
            ori = s[3]
            prevOri = prevS[3]
            blockOri = s[7]
        else:
            prevPos = np.array(prevS[:3])
            pos = np.array(s[:3])
            blockPos = np.array(s[6:9])
            prevBlock = np.array(prevS[6:9])
            ori = s[5]
            prevOri = prevS[5]
            blockOri = s[11]
        return prevPos, pos, blockPos, prevBlock, ori, prevOri, blockOri

    def rewardFunction(self, s, a):
        s = s.ravel()
        prevS = self.prev["S"].ravel()
        prevPos, pos, blockPos, prevBlock, ori, prevOri, blockOri = self.unpack(prevS, s)
        res = self.checkPhase(pos, blockPos, ori, blockOri, self.phase)
        if res[0] != 0:
            return res
        return self.getAux(pos, prevPos, blockPos, prevBlock, ori, prevOri, self.phase)
    
    def receiveState(self, msg):
        floats = vrep.simxUnpackFloats(msg.data)
        self.goal = np.array(floats[-4:-1])
        fail = floats[-1]
        restart = 0
        floats = floats[:self.s_n]

        s = (np.array(floats)).reshape(1,-1)
        a = (self.sendAction(s))

        if type(self.prev["S"]) == np.ndarray:
            r, restart = self.rewardFunction(s,self.prev['A'])
            if r == 5:
                print(" #### Phase ", self.phase, "  Complete!")
                self.phase += 1
            self.agent.store(self.prev['S'], self.prev["A"], np.array([r]).reshape(1, -1), s, a, restart)
            self.currReward += r

        if self.trainMode:
            loss = self.agent.train()

        self.prev["S"] = s
        self.prev["A"] = a
        self.restartProtocol(restart or fail)

        return 
    
    def restartProtocol(self, restart):
        if restart == 1:
            print('Results:     Cumulative Reward: ', self.currReward, '    Steps: ', self.agent.totalSteps)
            print("")
            for k in self.prev.keys():
                self.prev[k] = None
            self.goal = 0
            if self.currReward != 0:
                self.rewards.append(self.currReward)
            self.currReward = 0
            self.phase = 1
            self.agent.reset()

    ######### POST TRAINING #########
    def postTraining(self):
        #valueOnly = True if self.a == "argmax" else False
        self.plotRewards()
        self.plotLoss(False, 'Loss Over Iterations w/ Moving Average', "Actor Loss over Iterations w/ Moving Average")
        #self.agent.saveModel()
    
    def plotLoss(self, valueOnly = False, title1 = "Critic Loss over Iterations", title2 = "Actor Loss over Iterations"):
        x = range(len(self.agent.valueLoss))
        plt.plot(x, self.agent.valueLoss)
        plt.title(title1)
        plt.legend()
        window= np.ones(int(15))/float(15)
        line = np.convolve(self.agent.valueLoss, window, 'same')
        plt.plot(x, line, 'r')
        grid = True
        plt.show()
        if not valueOnly:
            x = range(len(self.agent.actorLoss))
            window = np.ones(int(15))/float(15)
            line = np.convolve(self.agent.actorLoss, window, 'same')
            plt.plot(x, line, 'r')
            plt.plot(x, self.agent.actorLoss)
            plt.title(title2)
            plt.show()
    
    def plotRewards(self):
        print(len(self.rewards))
        x = range(len(self.rewards))
        plt.plot(x, self.rewards)
        plt.title("Rewards Over Episodes w/ Moving Average")
        plt.legend()
        window= np.ones(int(15))/float(15)
        lineRewards = np.convolve(self.rewards, window, 'same')
        plt.plot(x, lineRewards, 'r')
        grid = True
        plt.show()