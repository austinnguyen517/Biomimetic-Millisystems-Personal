#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import math 
from network import Network
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt
from agent import Agent

class MADDPGAgent(Agent):
    def __init__(self, params):
        super(MADDPGAgent, self).__init__(params)

        actorParams = params['actorParams']
        actorTrain = params['actorTrain']

        self.discount = actorTrain['gamma']
        self.weight_loc = actorTrain['alpha1']
        self.weight_vel = actorTrain['alpha2']
        self.weight_agents = actorTrain['lambda']
        self.horizon = actorTrain['horizon']
        self.expSize = actorTrain['buffer']
        self.exploration = actorTrain['explore']

        self.own_n = actorParams['own_n']
        self.u_n = actorParams['output_n']
        self.prob = actorParams['prob']

        self.actor = Network(actorParams, actorTrain)
        self.critic = self.valueNet

        self.replayFeatures = 2*self.state_n + self.u_n*2 + 1 
        self.experience = np.zeros((self.expSize, self.replayFeatures))

        self.sigmoid = nn.Sigmoid()
        self.actorLoss = []
    
        while(not self.stop):
            x = 1+1

    def receiveState(self, message):
        floats = vrep.simxUnpackFloats(message.data)
        self.goalPosition = np.array(floats[-3:])
        state = (np.array(floats[:self.state_n])).reshape(1,-1)
        if self.startDistance == 0:
            pos = state[:, 3:6].ravel()
            self.startDistance = np.sqrt(np.sum(np.square(pos - self.goalPosition)))
        for i in range(self.agents_n - 1):
            '''TODO: receive the observations and ADD GAUSSIAN NOISE HERE PROPORTIONAL TO DISTANCE. Concatenate to the state'''
        action = (self.sendAction(state)).reshape(1,-1)
        if type(self.prevState) == np.ndarray:
            r = np.array(self.rewardFunction(state, action)).reshape(1,-1)
            self.experience[self.dataSize % self.expSize] = np.hstack((self.prevState, self.prevAction[:, :self.u_n], r, state, action[:, :self.u_n]))
            self.dataSize += 1
        self.prevState = state
        self.prevAction = action
        self.train()
        return 

    def train(self):
        if self.dataSize >= self.batch_size and self.dataSize >= self.horizon:
            choices = np.random.choice(min(self.expSize, self.dataSize), self.batch_size) 
            data = self.experience[choices]
            r = data[:, self.state_n + self.u_n: self.state_n + self.u_n + 1] #ASSUMING S, A, R, S', A' structure for experience 
            targets = torch.from_numpy(r) + self.discount * self.critic.predict(data[:, -(self.state_n + self.u_n): ])
            loss = self.critic.train_cust(data[:, :self.state_n + self.u_n], targets)
            self.valueLoss.append(loss)

            index = np.random.randint(0, self.experience.shape[0] - self.horizon)
            data = self.experience[index: index + self.horizon]
            stateAction = data[:, :(self.state_n+self.u_n)]
            criticValue = self.critic.predict(stateAction)
            states = data[:, :self.state_n]
            actions = data[:, self.state_n: self.state_n + self.u_n]
            loss = self.actor.train_cust(states, actions, criticValue)
            self.actorLoss.append(loss)