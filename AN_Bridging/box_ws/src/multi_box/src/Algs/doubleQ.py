#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import math 
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt

from torch.distributions import Categorical
from Networks.network import Network
from agent import Agent
from Networks.dualNetwork import DualNetwork
from utils import positiveWeightSampling
from Buffers.CounterFactualBuffer import Memory

'''Double DQN with priority sampling based on TD error. Possible to have dual networks for advantage and value'''

class DoubleQ(Agent):
    def __init__(self, params, name, task, load_path=None):
        super(DoubleQ, self).__init__(params, name, task)
        self.dual = self.vPars['dual']
        if self.trainMode:
            if self.dual:
                self.tarNet = DualNetwork(self.vPars, self.vTrain)
                self.valueNet = DualNetwork(self.vPars, self.vTrain)
            else:
                self.tarNet = Network(self.vPars, self.vTrain)
                self.valueNet = Network(self.vPars, self.vTrain)
                for target_param, param in zip(self.tarNet.parameters(), self.valueNet.parameters()):
                    target_param.data.copy_(param.data)
        else:
            self.valueNet = Network(self.vPars, self.vTrain)
            paths = ['/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/hierarchical_q_policy2.txt']
            if not load_path:
                self.valueNet = []
                for path in paths:
                    self.valueNet.append(Network(self.vPars, self.vTrain))
                    self.valueNet[-1].load_state_dict(torch.load(path))
            else:
                self.valueNet.load_state_dict(torch.load(load_path))
        self.out_n = self.vPars['neurons'][-1]
        self.replaceCounter = 0
        self.valueLoss = []
        self.avgLoss = 0
        self.expSize =self.vTrain['buffer']
        self.exp = Memory(size = self.expSize)

        self.priority = self.vTrain['priority']
        self.priorities = []
        self.beta = .5
        self.alpha = .7

        self.double = self.vTrain['double']
        if 'noise' in self.vTrain:
            self.noise = self.vTrain['noise']
        else:
            self.noise = 0
        

        task.initAgent(self)
    
        if not load_path:
            while(not self.stop):
                x = 1+1
            task.postTraining()

    def saveModel(self):
        torch.save(self.valueNet.state_dict(), '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/hierarchical_q_policy2.txt')
        pass
    
    def store(self, s, a, r, sprime, aprime, done):
        self.exp.push(s, a, r, 1-done, aprime, sprime)
        if len(self.priorities) < self.expSize:
            self.priorities.append(1)
        else:
            self.priorities = self.priorities[1:]
            self.priorities.append(1)
    
    def get_q(self, s):
        if type(self.valueNet) == list:
            model_index = np.random.randint(len(self.valueNet))
            net = self.valueNet[model_index]
        else:
            net = self.valueNet
        q = net(torch.FloatTensor(s))
        q = q.detach()
        return q
    
    def get_action(self, s, testing_time=False, probabilistic=False):
        i = np.random.random()
        if i < self.explore and self.trainMode and not testing_time:
            index = np.random.randint(self.out_n)
        else:
            q = self.get_q(s)
            print(q)
            if probabilistic:
                q = q.numpy()
                probs = np.exp(q)
                probs = probs / np.sum(probs)
                index = np.random.choice(q.size, p=probs.ravel())
            else:
                index = np.argmax(q.numpy())  
        self.explore = max(0, self.explore * .9997)
        return index
    
    def get_q_and_q_tar(self, states, actions, nextStates, rewards, masks):
        qValues = self.valueNet(torch.FloatTensor(states).squeeze(1)) #pass in. Processing implied
        q = torch.gather(qValues, 1, torch.LongTensor(actions).unsqueeze(1)) #get q values of actions  
        qnext  = self.tarNet(torch.FloatTensor(nextStates))
        qnext = qnext.squeeze(1).detach() #pass in

        if self.double: 
            qNextDouble = self.valueNet(torch.FloatTensor(nextStates))
            qNextDouble = qNextDouble.squeeze(1).detach() #pass in
            qnext = torch.gather(qnext, 1, torch.LongTensor(qNextDouble.argmax(1).unsqueeze(1)))
            qtar = torch.FloatTensor(rewards).squeeze(1) + self.discount * torch.Tensor(masks).unsqueeze(1) * qnext
        else:
            qtar = torch.FloatTensor(rewards) + self.discount * torch.Tensor(masks).unsqueeze(1) * qnext.max(1)[0].view(self.batch_size, 1) #calculate target
        return q, qtar

    def train(self, override=False):
        if len(self.exp) >= 500 or override:
            if self.priority:
                loss = 0
                weights = []
                errors = []
                assert len(self.priorities) == len(self.exp)
                for i in range(self.batch_size):
                    probs = np.array([math.pow(p, self.alpha) for p in self.priorities])
                    probs = probs / np.sum(probs)
                    choice = np.random.choice(len(self.priorities), p=probs, size=1)
                    weights.append(math.pow(len(self.priorities) * self.priorities[int(np.asscalar(choice))], -self.beta))
                    states, actions, rewards, masks, _, nextStates, _, _,_ = self.exp.get_transitions(choice)
                    q, qtar = self.get_q_and_q_tar(states, actions, nextStates, rewards, masks)
                    td = qtar - q
                    self.priorities[int(np.asscalar(choice))] = abs(td[:, 0])
                    errors.append(self.valueNet.get_loss(q, qtar))
                max_weight = max(weights)
                weights = [w/max_weight for w in weights]
                val_loss = sum([w * e for w,e in zip(weights, errors)])
                    
            else:
                states, actions, rewards, masks, _ , nextStates, _,_,_ = self.exp.sample(batch = self.batch_size)

                if self.replaceCounter % 200 == 0:
                    self.tarNet.load_state_dict(self.valueNet.state_dict())
                    self.replaceCounter = 0

                if self.noise:
                    states = np.array(states)
                    states = states + np.random.normal(0, self.noise, states.shape)

                q, qtar = self.get_q_and_q_tar(states, actions, nextStates, rewards, masks)
                val_loss = self.valueNet.get_loss(q, qtar)

            self.valueNet.optimizer.zero_grad()
            val_loss.backward()
            self.valueNet.optimizer.step()

            self.replaceCounter += 1
            self.totalSteps += 1
            return val_loss
