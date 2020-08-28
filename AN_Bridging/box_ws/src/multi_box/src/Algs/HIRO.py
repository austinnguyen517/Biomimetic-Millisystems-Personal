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
from Buffers.CounterFeudalBuffer import Memory
from collections import namedtuple
from utils import OUNoise as Noise

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
Transition = namedtuple('Transition', ('state', 'action', 'reward', 'mask', 'next_state', 'local','goal', 'next_goal'))

class HIRO(object):
    def __init__(self, params, name, task):
        self.name           = name
        self.task           = task

        self.vPars          = params['valPars']
        self.vTrain         = params['valTrain']
        self.mPars          = params['mPars']
        self.mTrain         = params['mTrain']
        self.wPars          = params['actPars']
        self.wTrain         = params['actTrain']
        self.w_vPars        = params['w_vPars']
        self.w_vTrain       = params['w_vTrain']

        self.agents         = params['agents']
        self.pubs = {}
        for key in self.agents.keys():
            bot             = self.agents[key]
            self.pubs[key]  = rospy.Publisher(bot['pub'], Vector3, queue_size = 1)
        rospy.Subscriber("/finished", Int8, self.receiveDone, queue_size = 1)

        self.valueLoss      = []

        self.manager        = Network(self.mPars, self.mTrain)
        self.m_critic       = Network(self.vPars, self.vTrain) 
        self.m_critic_target= Network(self.vPars, self.vTrain)
        self.worker         = Network(self.wPars, self.wTrain)
        self.w_critic       = Network(self.w_vPars, self.w_vTrain)
        self.w_critic_target= Network(self.w_vPars, self.w_vTrain)

        self.m_discount     = self.vTrain['m_gamma']
        self.w_discount     = self.vTrain['w_gamma']
        self.lr             = self.vTrain['lr']
        self.trainMode      = self.vPars['trainMode']
        self.step           = self.vTrain['step']
        self.stop           = False
        self.c              = self.mTrain['c']
        self.tau            = .005
        self.noise          = Noise(self.manager.neurons[-1], theta = .4, max_sigma = .2, min_sigma = 0, decay = 1)

        self.exp            = Memory()
        self.temp           = []
        self.totalSteps     = 0
        self.soft           = nn.Softmax(dim=1)

        self.reset()

        task.initAgent(self)

        while(not self.stop):
            x = 1+1

        task.postTraining()

    def receiveDone(self, message):
        if message.data  == 1: #all iterations are done. Check manager.py
            self.stop = True
        if message.data == 2: #timed out. Check manager.py
            self.task.restartProtocol(restart = 1)

    def get_action(self, s, s_w = None):
        s = torch.FloatTensor(s)
        if self.iteration % self.c == 0:
            self.goal = self.manager(s)
            noise = torch.FloatTensor(self.noise.get_noise())
            self.goal += noise
        else:
            self.goal = self.prevState[:,:2] + self.goal - s[:,:2]

        self.temp_second = self.temp_first
        self.temp_first = self.goal
        self.prevState = s
        s = s[:,:6]
        inpt = torch.cat((s, self.goal), dim=1)
        policy = self.worker(inpt)
        policy = self.soft(policy)
        choice = np.asscalar(self.choose(policy))
        self.iteration += 1
        return choice #single env

    def choose(self, policies):
        m = Categorical(policies)
        action = m.sample()
        action = action.data.cpu().numpy()
        return action
    
    def saveModel(self):
        pass

    def store(self, s, a, r, sprime, aprime, done):
        if self.temp_second != None:
            self.temp.append(Transition(s, a, r, 1-done, sprime, None, self.temp_second.detach().numpy(), self.goal.detach().numpy()))
            if self.iteration % self.c == 1 and self.iteration != 1: # remember, we push at 1 because we incremented in get_action
                self.temp = Transition(*zip(*self.temp))
                self.exp.push(self.temp) # store into exp
                self.temp = []


    def reset(self):
        self.iteration = 0
        self.temp_first, self.temp_second = (None, None)
        self.prevState = None
        self.temp = []
        return 
    
    def generateSamples(self, goals, states, next_states):
        next_states = next_states[:, :2]
        states = states[:, :2]
        candidates = (next_states - states).unsqueeze(0)
        candidates = torch.cat((candidates, goals.unsqueeze(0)), dim=0)
        normal = Normal(next_states - states, torch.ones(next_states.size()) / 2)
        sampled = normal.sample((8,))
        candidates = torch.cat((candidates, sampled), dim=0)
        # return shape (# candidates, batch_size, dimensions of goal)
        return candidates
    
    def getTransitions(self, initial_goals, states, next_states):
        # initial_goals shape: (# candidates ,batch_size, dimensions of goal)
        # states shape: (batch_size, c, dimensions of state)
        states = states[:,:, :2]
        next_states = next_states[:,:,:2]
        goals = [initial_goals.unsqueeze(0)]
        for c in range(self.c - 1):
            prev = goals[-1].squeeze(0)
            curr = states[:, c, :] + prev - next_states[:,c,:] # broadcast. This should take shape of initial_goals 
            goals.append(curr.unsqueeze(0))
        goals = torch.cat(goals, dim=0)
        goals = goals.transpose(0,1)
        goals = goals.transpose(1,2)
        # return shape (# candidates, batch_size, c, dimensions of goal)
        return goals
    
    def getProbabilities(self, transitions, states, actions):
        # transitions shape (# candidates, batch_size, c, dimensions of goal)
        # states shape: (batch_size, c, dimensions of state)
        # actions shape: (batch_size, c)
        states = states[:, :, :6]
        states = states.unsqueeze(0)
        size = states.size()
        states = states.expand(transitions.size()[0], size[1], size[2], size[3])
        inpt = torch.cat((states, transitions), dim=3)
        soft = nn.Softmax(dim = 3)
        actions = actions.expand(transitions.size()[0], actions.size()[0], actions.size()[1]).unsqueeze(3)
        probs = soft(self.worker(inpt)).gather(3, actions.long()).squeeze(3)
        probs = torch.prod(probs, dim=2)
        # return shape (# candidates, batch_size) of probabilities
        return probs

    def train(self):
        if len(self.exp) > 300:

            groups = self.exp.sample(self.step) # sample groupings of samples
            m_states = torch.cat(map(lambda g: torch.Tensor(g.state[0]), groups), dim=0)
            m_next_states = torch.cat(map(lambda g: torch.Tensor(g.next_state[-1]), groups), dim=0)
            m_goals = torch.cat(map(lambda g: torch.Tensor(g.goal[0]), groups), dim=0) 
            m_rewards = torch.Tensor(map(lambda g: sum(g.reward), groups)).squeeze(2)
            m_masks = torch.Tensor(map(lambda g: g.mask[-1], groups)).unsqueeze(1)

            w_states = torch.cat(map(lambda g: torch.Tensor(g.state).unsqueeze(0), groups), dim=0).squeeze()
            w_next_states = torch.cat(map(lambda g: torch.Tensor(g.next_state).unsqueeze(0), groups), dim=0).squeeze()
            w_actions = torch.cat(map(lambda g: torch.Tensor(g.action).unsqueeze(0), groups), dim=0)

            candidates = self.generateSamples(m_goals, m_states, m_next_states)
            cand_transitions = self.getTransitions(candidates, w_states, w_next_states)
            probs = self.getProbabilities(cand_transitions, w_states, w_actions)
            cand_indices = probs.argmax(dim=0).unsqueeze(0).unsqueeze(2)
            cand_indices = cand_indices.expand(cand_indices.size()[0], cand_indices.size()[1], candidates.size()[2])
            m_goals = candidates.gather(0, cand_indices).squeeze() #size: (batch_size, dimension of goals)
 
            states = []
            actions = []
            next_states = []
            masks = []
            goals = []
            next_goals = []
            for g in groups:
                states.append(torch.Tensor(g.state).squeeze()[:, :6])
                actions.append(torch.Tensor(g.action).squeeze())
                next_states.append(torch.Tensor(g.next_state).squeeze()[:, :6])
                masks.append(torch.Tensor(g.mask).squeeze())
                goals.append(torch.Tensor(g.goal).squeeze())  
                next_goals.append(torch.Tensor(g.next_goal).squeeze())

            states = torch.cat(states, dim=0)
            actions = torch.cat(actions, dim=0).unsqueeze(1)
            next_states = torch.cat(next_states, dim=0)
            masks = torch.cat(masks, dim=0).unsqueeze(1)
            goals = torch.cat(goals, dim=0) 
            next_goals = torch.cat(next_goals, dim=0)
            rewards = -torch.norm(states[:,:2] + goals - next_states[:,:2], dim=1).unsqueeze(1)

            # Manager critic
            q = self.m_critic(torch.cat((m_states, m_goals), dim=1))
            m_next_actions = self.manager(m_next_states)
            q_tar = m_rewards + self.m_discount * self.m_critic_target(torch.cat((m_next_states, m_next_actions), dim=1))
            loss = self.m_critic.get_loss(q, q_tar.detach())
            self.m_critic.optimizer.zero_grad()
            loss.backward()
            self.m_critic.optimizer.step()

            # Manager actor
            new_actions = self.manager(m_states)
            q = self.m_critic(torch.cat((m_states, new_actions), dim=1))
            loss = -q.mean()
            self.manager.optimizer.zero_grad()
            loss.backward()
            self.m_critic.optimizer.step()

            # Worker critic
            q = self.w_critic(torch.cat((states, goals), dim=1)).gather(1, actions.long())
            next_actions = self.worker(torch.cat((next_states, next_goals), dim=1))
            next_actions = self.choose(self.soft(next_actions))
            q_tar = rewards + self.w_discount * masks * self.w_critic_target(torch.cat((next_states, next_goals), dim=1)).gather(1, torch.Tensor(next_actions).long().unsqueeze(1))
            loss = self.w_critic.get_loss(q, q_tar.detach())
            self.w_critic.optimizer.zero_grad()
            loss.backward()
            self.w_critic.optimizer.step()

            # Worker actor 
            new_actions = self.worker(torch.cat((states[:,:6], goals), dim=1))
            policy = self.soft(new_actions)
            new_actions = self.choose(policy)
            q = self.w_critic(torch.cat((states, goals), dim=1))
            q = q.gather(1, torch.Tensor(new_actions).long().unsqueeze(1))
            loss = -q.mean()
            self.worker.optimizer.zero_grad()
            loss.backward()
            self.worker.optimizer.step()
    
            for target_param, param in zip(self.m_critic_target.parameters(), self.m_critic.parameters()):
                target_param.data.copy_(target_param.data * (1.0 - self.tau) + param.data * self.tau)
            for target_param, param in zip(self.w_critic_target.parameters(), self.w_critic.parameters()):
                target_param.data.copy_(target_param.data * (1.0 - self.tau) + param.data * self.tau)
            
            # Push updated replay entires into replay

            for i, goal in enumerate(m_goals):
                curr_group = groups[i]
                curr_goal = goal.unsqueeze(0).detach().numpy()
                inserts = (curr_goal)
                for j in range(self.c - 1):
                    curr_goal = curr_group.state[j][:,:2].reshape(1,-1) + curr_goal - curr_group.next_state[j][:,:2].reshape(1,-1)
                    inserts = inserts + (curr_goal)
                curr_group._replace(goal=inserts)
                self.exp.push(curr_group)

            self.totalSteps += 1

            return loss




    
