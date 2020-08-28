#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import math 
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt
import torch.optim as optim

from Networks.network import Network
from Networks.softNetwork import SoftNetwork
from agent import Agent
from Buffers.CounterFactualBuffer import Memory 

cuda_avail = torch.cuda.is_available()
device = torch.device("cuda" if cuda_avail else "cpu")

class SAC(Agent):
    def __init__(self, params, name, task):
        super(SAC, self).__init__(params, name, task)
        self.aPars      = params['actPars']
        self.aTrain     = params['actTrain']
        self.qPars      = params['qPars']
        self.qTrain     = params['qTrain']
        if self.trainMode:
            self.QNet = Network(self.qPars, self.qTrain).to(device)
            self.VNet = Network(self.vPars, self.vTrain).to(device)
            self.VTar = Network(self.vPars, self.vTrain).to(device)
            self.policyNet = SoftNetwork(self.aPars, self.aTrain).to(device)
        else:
            print('Not implemented')

        for target_param, param in zip(self.VTar.parameters(), self.VNet.parameters()):
            target_param.data.copy_(param)

        self.expSize    = self.vTrain['buffer']
        self.actions    = self.aPars['neurons'][-1]
        self.state      = self.aPars['neurons'][0]
        self.exp        = ReplayBuffer(self.expSize, self.actions, np.float32, self.state, np.float32)

        task.initAgent(self)
    
        while(not self.stop):
            x = 1+1
        task.postTraining()

    def load_nets(self):
        pass
    
    def saveModel(self):
        pass
    
    def get_action(self, s):
        action, _ , _, _, _= self.policyNet(torch.FloatTensor(s))
        action = np.ravel(action.detach().numpy())
        return action

    def send_to_device(self, s, a, r, next_s, d):
        s = torch.FloatTensor(s).to(device)
        a = torch.FloatTensor(a).to(device)
        r = torch.FloatTensor(r).unsqueeze(1).to(device)
        next_s = torch.FloatTensor(next_s).to(device)
        d = torch.FloatTensor(np.float32(d)).unsqueeze(1).to(device)
        return s, a, r, next_s, d
        
    def train(self):
        if len(self.exp) > 750:
            s, a, r, next_s, d = self.exp.sample_batch(self.batch_size)
            s, a, r, next_s, d = self.send_to_device(s, a, r, next_s, d)

            q = self.QNet(torch.cat([s, a], dim = 1))
            v = self.VNet(s)
            new_a, log_prob, z, mean, log_std = self.policyNet(s)

            target_v = self.VTar(next_s)

            next_q = r + (1 - d) * self.discount * target_v
            q_loss = self.QNet.get_loss(q, next_q.detach())

            new_q = self.QNet(torch.cat([s, new_a], dim=1))
            next_v = new_q - log_prob * self.alpha
            v_loss = self.VNet.get_loss(v, next_v.detach())

            target = new_q - v
            actor_loss = (log_prob * (log_prob*self.alpha - target).detach()).mean()

            mean_loss = 1e-3 * mean.pow(2).mean()
            std_loss = 1e-3 * log_std.pow(2).mean()
            actor_loss += mean_loss + std_loss

            self.VNet.optimizer.zero_grad()
            v_loss.backward()
            self.VNet.optimizer.step()

            self.QNet.optimizer.zero_grad()
            q_loss.backward()
            self.QNet.optimizer.step()

            self.policyNet.optimizer.zero_grad()
            actor_loss.backward()
            self.policyNet.optimizer.step()

            for target_param, param in zip(self.VTar.parameters(), self.VNet.parameters()):
                target_param.data.copy_(target_param.data * (1.0 - 5*1e-3) + param.data * 5*1e-3)

            self.totalSteps += 1

        
            
