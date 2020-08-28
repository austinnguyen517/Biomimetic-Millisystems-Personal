#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import torch.optim as optim
import math 
import rospy
import torch.nn.functional as F
from torch.distributions import Categorical, Normal
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt

from agent import Agent
from Networks.network import Network
from Networks.feudalNetwork import FeudalNetwork
from Buffers.CounterFeudalBuffer import Memory
from collections import namedtuple

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
Transition = namedtuple('Transition', ('states', 'actions', 'rewards', 'next_state', 'local','goals'))

# Other considerations:
#   - Latent space for actor instead of manually reduced space. Use auxiliary reward prediction for feature extraction


class CounterFeudal(object):
    def __init__(self, params, name, task):
        self.name           = name
        self.task           = task
        self.vTrain         = params['valTrain'] # Counterfactual network
        self.vPars          = params['valPars']
        self.aTrain         = params['actTrain'] # Local Actors
        self.aPars          = params['actPars']
        self.m_params       = params['m_pars'] # Manager
        self.m_train        = params['m_train']
        self.local_vPars    = params['local_pars'] # Local values
        self.local_vTrain   = params['local_train']
        self.agents         = params['agents'] # Agents

        self.pubs = {}
        for key in self.agents.keys():
            bot             = self.agents[key]
            self.pubs[key]  = rospy.Publisher(bot['pub'], Vector3, queue_size = 1)
        rospy.Subscriber("/finished", Int8, self.receiveDone, queue_size = 1)

        self.tau            = self.vPars['tau']
        self.trainMode      = self.vPars['trainMode']
        self.batch_size     = self.vTrain['batch']
        self.td_lambda      = .8

        self.c              = self.m_params['c']
        self.w_discount     = self.vTrain['gamma']
        self.m_discount     = self.m_train['gamma']
        self.prevState      = None
        self.soft           = nn.Softmax(dim=1)

        self.exp            = Memory()
        self.valueLoss      = []
        self.actorLoss      = []
        self.temp           = []
        self.goal_temp1     = None 
        self.goal_temp2     = None
        self.iteration      = 0
        self.totalSteps     = 0

        self.counter_critic, self.counter_target = (Network(self.vPars, self.vTrain), Network(self.vPars, self.vTrain))
        self.manager = Network(self.m_params, self.m_train) # manager
        self.actor = Network(self.aPars, self.aTrain) # actor
        self.critic, self.target = (Network(self.local_vPars, self.local_vTrain), Network(self.local_vPars, self.local_vTrain))

        for target_param, param in zip(self.target.parameters(), self.critic.parameters()):
            target_param.data.copy_(param.data)
        for target_param, param in zip(self.counter_target.parameters(), self.counter_critic.parameters()):
            target_param.data.copy_(param.data)       

        self.reset()

        task.initAgent(self)
        self.stop = False
        while(not self.stop):
            x = 1+1

        task.postTraining()

    def receiveDone(self, message):
        if message.data  == 1: #all iterations are done. Check manager.py
            self.stop = True
        if message.data == 2: #timed out. Check manager.py
            self.task.restartProtocol(restart = 1)

    def get_action(self, s_true, s_split):
        if self.iteration % self.c == 0: 
            self.goal = [self.manager(torch.FloatTensor(s)) for s in s_split]
        else: # Use goal transition 
            self.goal = [torch.FloatTensor(self.prevState[i][:,:2]) + g - torch.FloatTensor(s_split[i][:,:2]) for i, g in enumerate(self.goal)]

        self.goal_temp2 = self.goal_temp1 
        self.goal_temp1 = self.goal

        policies = []
        for i, s in enumerate(s_split):
            inpt = torch.cat((s[:,:6], self.goal[i]), dim=1)
            policies.append(self.soft(self.actor(inpt)))  
        act_indices = [self.choose(policy) for policy in policies]
        actions = [self.actionMap[index] for index in act_indices]

        self.prevState = s_split
        self.iteration += 1

        return act_indices

    def choose(self, policies):
        m = Categorical(policies)
        action = m.sample()
        action = action.data.cpu().numpy()
        if action.size == 1:
            return np.asscalar(action)
        return torch.Tensor(action).unsqueeze(1)
    
    def saveModel(self):      
        pass

    def store(self, s, a, r, sprime, aprime, done, s_w):
        self.temp.append(Transition(s, a, r, sprime, s_w, self.goal_temp2))
        if self.iteration % self.c == 1 and self.iteration != 1: # remember, we push at 1 because we incremented in get_action
            self.exp.push(self.temp) # store into exp
            self.temp = []

    def reset(self):
        self.train(True)
        self.iteration = 0
        self.temp = []
        self.goal_temp1, self.goal_temp2 = (None, None)
        return 

    def zipStack(self, data):
        data        = zip(*data)
        data        = [torch.stack(d).squeeze().to(device) for d in data]
        return data

    def train(self, episode_done = False): 
        if len(self.exp) > self.batch_size:
            # UNPACK REPLAY
            groups = self.exp.sample(self.batch_size)
            
            # for each agent:
                # Extract manager samples by: taking first true state, taking first goal, sum rewards, last next state of each group
                # replace the goal: Sample new goals according to the paper 
                # for each of the goals, get goal transitions using worker state_transitions. 
                # Pass goal transitions concatenated with worker_states to get policy distribution
                # gather all actions according to the replay actions 
                # multiply probabilities across time
                # choose the goal index that has highest probability of all and replace each of the groups with the new goal and transition
            
            
            # Train counterfactual
                # Same as continuous counterfactual learning. Use montecarlo for each of the manager transitions 
            
            # Train manager 
                # Same as continuous counterfactual learning 
            
            # Train value
                # Same as normal value learning 
            
            # Train worker
                # Use advantage actor critic but through experience replay
            
            # store the groupings back into the replay
            print('yes')
        return 1

    


    
