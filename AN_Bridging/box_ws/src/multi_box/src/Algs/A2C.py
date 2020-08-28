#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
from torch.distributions import Normal
import math 
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt

from Networks.network import Network
from Networks.A2CNetwork import A2CNetwork
from agent import Agent
from utils import positiveWeightSampling

'''Advantage Actor Critic. On-policy.'''

class A2C(Agent):
    def __init__(self, params, name, task):
        super(A2C, self).__init__(params, name, task)
        if self.trainMode:
            self.aPars = params['actPars']
            self.aTrain = params['actTrain']
            self.value = Network(self.vPars, self.vTrain)
            self.policyNet = A2CNetwork(self.aPars, self.aTrain)
        else:
            self.policyNet = Network(self.aPars, self.aTrain)
            self.policyNet.load_state_dict(torch.load("/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/PolicyNet.txt"))

        self.exp = Replay(self.batch_size)
        self.replaceCounter = 0
        self.valueLoss = []
        self.actorLoss = []
        self.avgLoss = 0
        self.avgActLoss = 0

        task.initAgent(self)
    
        while(not self.stop):
            x = 1+1
        task.postTraining()
    
    def saveModel(self):
        #print("Network saved")
        pass

    def get_action(self):
        output = self.policyNet(torch.FloatTensor(s))
        action_mean = output[:, :int(out_n/2)]
        action_logstd = output[:, int(out_n/2):]
        action_std = torch.exp(action_logstd)
        action = (torch.normal(action_mean, action_std).detach().numpy()).ravel()
        return action
                
    def train(self):
        if self.dataSize == self.batch_size:
            self.totalSteps += 1    
            s, a, r, n_s, n_a, mask = self.exp.get_data()
            mask = torch.FloatTensor(np.where(mask  > .5, 0, 1)) #if fail, equal to 1 so set mask to 0

            #Critic update
            vTar = torch.FloatTensor(r) + self.discount * self.value(n_s).detach()
            v = self.value(s)
            loss = self.value.loss_fnc(v, vTar)
            self.value.optimizer.zero_grad()
            loss.backward()
            self.value.optimizer.step()
            self.avgLoss += loss

            #Policy update:
            advantage = (vTar - v).detach()
            out = self.policyNet(s)
            mean = out[:, :int(self.out_n)]
            log_std = out[:, int(self.out_n):]
            log_prob = Normal(mean, torch.exp(log_std)).log_prob(torch.FloatTensor(a))
            entropy = -torch.sum(torch.exp(log_prob) * log_prob)
            grad = -torch.sum(log_prob * advantage) + .01 * entropy
            self.policyNet.optimizer.zero_grad()
            grad.backward()
            self.policyNet.optimizer.step()
            self.avgActLoss += grad            

            #iteration updates
            self.trainIt += 1
            self.dataSize = 0
            
