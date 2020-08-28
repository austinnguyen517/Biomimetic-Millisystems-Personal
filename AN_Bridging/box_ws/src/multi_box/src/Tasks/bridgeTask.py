#! /usr/bin/env python

#! /usr/bin/env python

from task import Task 
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

class BridgeTask(Task):
    def __init__(self):
        super(BridgeTask, self).__init__()
        self.prev = {"S": None, "A": None}
        self.actionMap        = {0: (-2,-1), 1:(-1,-2), 2:(-2,-2), 3:(1,2), 4:(2,2), 5:(2,1), 6: (-2, 2), 7: (2, -2)} 
        self.discrete = True
        self.currReward = 0
        self.rewards = []
        self.phase = 1


    def extractInfo(self):
        self.vTrain = self.agent.vTrain
        self.pubs = self.agent.pubs
        self.trainMode = self.agent.trainMode
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
            msg.x, msg.y, msg.z = (action[i][0], action[i][1], action[i][2])
            self.pubs[key].publish(msg)
        return ret
    
    def rewardFunction(self, s_n, a):
        tank, bridge = self.splitState(s_n.ravel())
        prevTank, prevBridge = self.splitState(self.prev['S'].ravel())
        if bridge[2] < .1 or tank[2] < .1:
            # THIS IS A TEST
            return (0, 1)
        if self.phase == 1:
            if bridge[0] > -.5: # TODO: Check this benchmark for bridge
                print( '## Phase 1 Complete ##')
                self.phase += 1
                return (5, 0)
            vel_r = ((tank[0] - prevTank[0]) + (bridge[0] - prevBridge[0])) * 2
            ori_r = -1 * (abs(tank[5]) + abs(bridge[5])) * .08
            r = vel_r + ori_r
        elif self.phase == 2:
            if tank[0] > -.4: # TODO: Check this benchmark for cross
                print(' ## Phase 2 Complete ##')
                self.phase += 1
                return (5, 0)
            vel_r = (tank[0] - prevTank[0])
            move_r = -1 * dist(prevBridge[:3], bridge[:3])
            r = vel_r + move_r
        elif self.phase == 3:
            if bridge[0] > -.3: # TODO: Check this benchmark for pull up
                print(' ## Success ##')
                return (10, 1)
            vel_r = (bridge[0] - prevBridge[0])
            r = vel_r 
        return (r, 0)

    
    def splitState(self, s):
        # Assumption: tanker first then bridge
        tank = np.hstack((np.array(s[:7]), np.array([self.phase])))
        bridge = np.hstack((np.array(s[7:13]), np.array([self.phase])))
        tank = np.hstack((tank, bridge[:2] - tank[:2], bridge[5:6]))
        bridge = np.hstack((bridge, tank[:2] - bridge[:2], tank[5:7]))
        return tank, bridge


    def receiveState(self, msg):
        floats = vrep.simxUnpackFloats(msg.data)
        fail = floats[-1]
        floats = floats[:-1]
        restart = 0
        first, second = self.splitState(floats)
        floats.append(self.phase)
        
        s = (np.array(floats)).reshape(1,-1)
        first = torch.FloatTensor(first).view(1,-1)
        second = torch.FloatTensor(second).view(1,-1)
        a = (self.sendAction(s, [first, second]))
        if type(self.prev["S"]) == np.ndarray:
            r, restart = self.rewardFunction(s, self.prev['A'])   
            self.agent.store(self.prev['S'], self.prev["A"], r, s, a, restart, [first, second]) 
            self.currReward += r
        self.prev["S"] = s
        self.prev["A"] = a
        s = s.ravel()
        if self.trainMode:
            self.agent.train()
        self.restartProtocol(fail or restart)
        return 

    def restartProtocol(self, restart):
        if restart == 1:
            print('Results:     Cumulative Reward: ', self.currReward, '    Steps: ', self.agent.totalSteps)
            print("")
            for k in self.prev.keys():
                self.prev[k] = None
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