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

from Networks.network import Network
from Networks.TD3Network import TD3Network
from agent import Agent
from utils import positiveWeightSampling as priority
from utils import OUNoise

'''Twin-delayed DDPG to curb Q value overestimation with clipped double Q-learning, Q value smoothing using noise and delayed policy updates for stability'''

class Twin_DDPG(Agent):
    def __init__(self, params, name, task):
        super(Twin_DDPG, self).__init__(params, name, task)
        self.aPars  = params['actPars']
        self.aTrain = params['actTrain']

        if self.trainMode:
            self.values     = [Network(self.vPars, self.vTrain), Network(self.vPars, self.vTrain)]
            self.policyNet  = TD3Network(self.aPars, self.aTrain)
            self.tarPolicy  = TD3Network(self.aPars, self.aTrain)

            if self.load:
                self.load_nets()

            self.tarPolicy.load_state_dict(self.policyNet.state_dict())
            self.tar = [Network(self.vPars, self.vTrain), Network(self.vPars, self.vTrain)]
            for i in range(len(self.values)):
                self.tar[i].load_state_dict(self.values[i].state_dict())
        else:
            self.policyNet = Network(self.aPars, self.aTrain)
            self.policyNet.load_state_dict(torch.load("/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/TD3_goal_policy2.txt"))

        self.base       = self.vTrain['baseExplore']
        self.step       = self.vTrain['decay']
        self.expSize    = self.vTrain['buffer']
        self.exp        = Replay(self.expSize)
        self.a          = self.vTrain['a']
        self.tau        = self.vPars['tau']
        self.smooth     = self.vTrain['smooth']
        self.clip       = self.vTrain['clip']
        self.delay      = self.vTrain['policy_delay']
        self.mean_range = self.aPars['mean_range']
        self.noise      = OUNoise(self.out_n, mu = 0, theta = .15, max_sigma = self.explore, min_sigma = self.base, decay = self.step)
        self.valueLoss  = []
        self.actorLoss  = []
        self.avgLoss    = 0
        self.avgActLoss = 0

        task.initAgent(self)
    
        while(not self.stop):
            x = 1+1
        task.postTraining()

    def load_nets(self):
        path = "/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/TD3_goal3_"
        self.policyNet.load_state_dict(torch.load(path + "policy.txt"))
        self.values[0].load_state_dict(torch.load(path + "Qvalue1.txt"))
        self.values[1].load_state_dict(torch.load(path + "Qvalue2.txt"))
    
    
    def saveModel(self):
        path = "/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/TD3_goal3_"
        torch.save(self.policyNet.state_dict(), path + "policy.txt")
        torch.save(self.values[0].state_dict(), path + "Qvalue1.txt")
        torch.save(self.values[1].state_dict(), path + "Qvalue2.txt")
        print("Network saved")
        pass

    def get_action(self):
        output = self.policyNet(torch.FloatTensor(s))
        i = np.random.random()
        if i < self.explore[0]:
            #add in exploration TODO: put in OU noise
            noise = torch.from_numpy(np.random.normal(0, self.explore[1], 2))
            output = output + noise
        output = output.float()
        return output[0]
        
    def train(self):
        if self.dataSize > 500 and self.trainMode:
            #iteration updates
            self.trainIt += 1
            self.totalSteps += 1

            #Unpack
            s, a, r, n_s, n_a, done = self.exp.get_data()
            noise = torch.FloatTensor(np.random.normal(0, self.smooth, n_a.shape))

            c = np.random.choice(min(self.dataSize, self.expSize), self.batch_size)

            s = torch.FloatTensor(s[c])
            a = torch.FloatTensor(a[c])
            r = torch.FloatTensor(r[c])
            n_s = torch.FloatTensor(n_s[c])
            done = torch.FloatTensor(done[c])
            n_a = self.tarPolicy(n_s).detach().numpy() 

            #target policy smoothing 
            n_a_ = n_a + torch.clamp(noise , -self.clip, self.clip)
            n_sa = torch.cat((n_s, n_a), dim = 1)
            qtar = torch.FloatTensor(r) + self.discount*(1-done)*torch.min(self.tar[0](n_sa).detach(), self.tar[1](n_sa).detach()) #pass in

            #Value update
            sa = torch.cat((s,a), dim = 1)
            for qnet in self.values:
                q = qnet(sa) 
                loss = qnet.loss_fnc(q, qtar)
                qnet.optimizer.zero_grad()
                loss.backward()
                qnet.optimizer.step()
                qnet.scheduler.step()
                self.avgLoss += loss / len(self.values) 

            #policy update
            if self.trainIt % self.delay == 0:
                act = self.policyNet(s)
                s_a = torch.cat((s, act), 1)
                q = self.values[0](s_a)
                policy_loss = -q.mean()

                self.policyNet.optimizer.zero_grad()
                policy_loss.backward()
                self.policyNet.optimizer.step()
                self.policyNet.scheduler.step()
                self.avgActLoss += policy_loss
        
                for target_param, param in zip(self.tarPolicy.parameters(), self.policyNet.parameters()):
                    target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)
                
                for i in range(len(self.values)):
                    for target_param, param in zip(self.tar[i].parameters(), self.values[i].parameters()):
                        target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)
            
