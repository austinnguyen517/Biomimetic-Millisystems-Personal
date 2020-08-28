#! /usr/bin/env python

#! /usr/bin/env python

from task import distance as dist 
from task import Task, dot, unitVector
import numpy as np 
import torch 
import torch.nn as nn
import vrep
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt

class MoveTask(Task):
    def __init__(self, action):
        super(MoveTask, self).__init__(action)

        self.prev = {"S": None, "A": None}
        self.actionMap = {0: (-2,-1), 1:(-1,-2), 2:(-2,-2), 3:(1,2), 4:(2,2), 5:(2,1), 6: (-2, 2), 7: (2, -2)} 
        self.restart = rospy.Publisher('/restart', Int8, queue_size = 1)
        rospy.Subscriber('/restart', Int8, self.restartCall, queue_size = 1)
   
        self.currReward, self.currIt, self.prevIt, self.goal = (0, 0, 0, 0)
        self.c = 30
        self.success = 5
        self.rewards = []

    def extractInfo(self):
        self.vTrain = self.agent.vTrain
        self.pubs = self.agent.pubs
        self.actions = self.agent.actions
        self.trainMode = self.agent.trainMode
        self.explore = self.agent.explore
        self.name = self.agent.name
        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 
 
    def sendAction(self, s):
        msg = Vector3()
        if self.a == "argmax":
            q = self.valueNet(s)
            i = np.random.random()
            if i < self.explore:
                index = np.random.randint(self.actions)
            else:
                q = q.detach().numpy()
                index = np.argmax(q)
            msg.x, msg.y = self.actionMap[index]
            action = np.array([index])
        if self.a == "p_policy":
            action, _ , _, _, _= self.policyNet(torch.FloatTensor(s))
            action = np.ravel(action.detach().numpy())
            msg.x, msg.y = (action[0], action[1])
        if self.a == "d_policy":
            output = self.policyNet(torch.FloatTensor(s).view(-1, s.size))
            i = np.random.random()
            output = output.detach().numpy() 
            if self.trainMode:
                noise = self.agent.noise.get_noise(t = 1)
                print(noise, '    ', output)
                output = np.clip(output + noise, -self.agent.mean_range, self.agent.mean_range)
            rav = np.ravel(output)
            msg.x = rav[0]
            msg.y = rav[1]
            self.pubs[self.name].publish(msg)
            return output
        self.pubs[self.name].publish(msg)
        return action.reshape(1,-1)
    
    def rewardFunction(self, s, a, s_n):
        currDist = dist(s_n,np.zeros(s_n.shape))
        if currDist < .5:
            return (1, 1)
        reg = .1 * np.sum(3 - np.abs(a)) if self.a != "argmax" else 0
        prev = self.prev['S']
        prevOri = unitVector(prev)
        ori = unitVector(s_n)
        r_ori = abs(ori[0]) - abs(prevOri[0]) 
        deltDist = 10* (dist(prev, np.zeros(prev.shape)) - dist(s_n, np.zeros(s_n.shape)))
        return ((deltDist + r_ori - reg)/self.success, 0)

    def receiveGoal(self, msg):
        goal = np.array(vrep.simxUnpackFloats(msg.data))
        self.goal = goal

    def receiveState(self, msg):
        s = np.array(vrep.simxUnpackFloats(msg.data))
        finish = 0  
        self.prevIt = self.currIt 

        a = (self.sendAction(s))

        if type(self.prev["S"]) == np.ndarray: 
            r, finish = self.rewardFunction(self.prev['S'], self.prev['A'], s)
            r = np.array(r).reshape(1,-1)
            if finish:
                print('#### SUCCESS!!!! ####')
            self.agent.store(self.prev['S'].reshape(1,-1), self.prev["A"], r, s.reshape(1,-1), a, finish)
            self.currReward += np.asscalar(r)

        if self.trainMode and len(self.agent.exp) >= self.agent.batch_size:
            self.agent.train()

        self.prev["S"] = s
        self.prev["A"] = a.reshape(1,-1)
        s = s.ravel()
        self.currIt += 1

        if self.currIt > self.c or finish:
            msg = Int8()
            msg.data = 1
            self.restart.publish(msg)
            return
        
    def restartCall(self, msg):
        if msg.data == 1:
            self.restartProtocol(1)
    
    def restartProtocol(self, restart): 
        if restart == 1:
            print('Results:     Cumulative Reward: ', self.currReward, '    Steps: ', self.agent.totalSteps)
            print("")
            for k in self.prev.keys():
                self.prev[k] = None
            if self.agent.trainIt > 0:
                self.agent.valueLoss.append((self.agent.avgLoss)/self.agent.trainIt)
                self.agent.avgLoss = 0 
                if self.a == 'p_policy' or self.a == 'd_policy':
                    self.agent.actorLoss.append((self.agent.avgActLoss)/self.agent.trainIt)  
                    self.agent.avgActLoss = 0 
                self.rewards.append(self.currReward)
            self.currIt, self.goal, self.agent.trainIt, self.currReward = (0,0,0,0)
            self.prevIt = self.currIt
            self.distances = []

    ######### POST TRAINING #########
    def postTraining(self):
        valueOnly = True if self.a == "argmax" else False
        self.plotLoss(valueOnly, "Value Loss over Iterations", "Actor Loss over Iterations")
        self.plotRewards()
        self.agent.saveModel()
        print("Total steps: ", self.agent.totalSteps)
    
    def plotLoss(self, valueOnly = False, title1 = "Critic Loss over Iterations", title2 = "Actor Loss over Iterations"):
        plt.plot(range(len(self.agent.valueLoss)), self.agent.valueLoss)
        plt.title(title1)
        plt.show()
        if not valueOnly:
            plt.plot(range(len(self.agent.actorLoss)), self.agent.actorLoss)
            plt.title(title2)
            plt.show()
    
    def plotRewards(self):
        x = range(len(self.rewards))
        #plt.plot(x, self.rewards)
        plt.title("Moving Average Rewards Over Episodes")
        plt.legend()
        window= np.ones(int(15))/float(15)
        lineRewards = np.convolve(self.rewards, window, 'same')
        plt.plot(x, lineRewards, 'r')
        grid = True
        plt.show()