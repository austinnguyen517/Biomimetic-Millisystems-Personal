#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import torch.optim as optim
import math 
import rospy
import time
import torch.nn.functional as F
from torch.distributions import Categorical
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt

from agent import Agent
from Networks.network import Network
from Networks.feudalNetwork import FeudalNetwork
from Buffers.CounterFactualBuffer import Memory
from collections import namedtuple, OrderedDict

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class Counter(object):
    def __init__(self, params, name, task):
        self.name           = name
        self.task           = task
        self.vTrain         = params['valTrain']
        self.vPars          = params['valPars']
        self.aTrain         = params['actTrain']
        self.aPars          = params['actPars']
        self.agents         = params['agents']

        self.pubs = OrderedDict()
        for key in self.agents.keys():
            bot             = self.agents[key]
            self.pubs[key]  = rospy.Publisher(bot['pub'], Vector3, queue_size = 1)
        rospy.Subscriber("/finished", Int8, self.receiveDone, queue_size = 1)

        self.valueLoss      = []
        self.actorLoss      = []

        self.h_state_n      = self.aPars['h_state_n']
        self.x_state_n      = self.aPars['x_state_n']
        self.u_n            = self.aPars['u_n']
        self.clip_grad_norm = self.aTrain['clip']
        self.homogenous     = self.aPars['share_params']

        self.critic         = Network(self.vPars, self.vTrain).to(device)
        self.target         = Network(self.vPars, self.vTrain).to(device)
        if self.homogenous:
            self.actor      = CounterActor(self.aPars, self.aTrain).to(device) 
        else:
            self.actor      = [CounterActor(self.aPars, self.aTrain) for i in range(len(self.agents))]

        for target_param, param in zip(self.target.parameters(), self.critic.parameters()):
            target_param.data.copy_(param.data)

        self.clip_grad_norm = self.aTrain['clip']
        self.trainMode      = self.vPars['trainMode']
        self.batch_size     = self.vTrain['batch']
        self.discount       = self.vTrain['gamma']
        self.temp_second    = None 
        self.temp_first     = None
        self.td_lambda      = 0 # TEST: this is because we are doing ER off-policy
        self.tau            = .01
        self.stop           = False
        self.trained        = False

        self.exp            = Memory()

        self.totalSteps     = 0

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

    def get_action(self, s_true, s_split):
        if self.homogenous:
            policy1 = self.actor(torch.FloatTensor(s_split[0]))
            a1 = np.asscalar(self.choose(policy1))
            policy2 = self.actor(torch.FloatTensor(s_split[1]))
            a2 = np.asscalar(self.choose(policy2))
        else:
            policy1 = self.actor[0](torch.FloatTensor(s_split[0]))
            a1 = self.choose(policy1)
            policy2 = self.actor[1](torch.FloatTensor(s_split[1]))
            a2 = self.choose(policy2)
        # THIS IS A TEST
        a1 = 0
        #print(policy1)
        #print(policy2)
        #print('')
        return [a1, a2]

    def choose(self, policies):
        m = Categorical(policies)
        action = m.sample()
        action = action.data.cpu().numpy()
        return action

    
    def saveModel(self):
        pass

    def store(self, s, a, r, sprime, aprime, done, local, next_local):
        self.exp.push(s, a, r, 1 - done, aprime, sprime, local, next_local, None)

    def reset(self):
        self.train(True)
        if self.trained:
            self.actor.eps = max(.05, self.actor.eps - .003)
        self.trained = False
        self.temp_first, self.temp_second = (None, None)
        self.h = [torch.zeros((1, 1, self.h_state_n)) for i in range(len(self.agents))]
        self.prevAction     = [-1,-1]
        return 

    def zipStack(self, data):
        data        = zip(*data)
        data        = [torch.stack(d).squeeze().to(device) for d in data]
        return data

    def get_lambda_targets(self, rewards, mask, gamma, target_qs):
        target_qs = target_qs.squeeze()
        ret = target_qs.new_zeros(*target_qs.shape)
        ret[-1] = rewards[-1] + target_qs[-1] * mask[-1]


        for t in range(ret.size()[0] - 2, -1,  -1):
            ret[t] = mask[t] * (self.td_lambda * gamma * ret[t + 1]) + (rewards[t] + (1 - self.td_lambda) * gamma * target_qs[t] * mask[t])
        return ret.unsqueeze(1)

    def train(self, episode_done = False):
        if len(self.exp) > self.batch_size:
            transition = self.exp.sample(self.batch_size)
            states = torch.squeeze(torch.Tensor(transition.state)).to(device)
            states_next = torch.squeeze(torch.Tensor(transition.next_state)).to(device) 
            actions = torch.Tensor(transition.action).float().to(device)
            rewards = torch.Tensor(transition.reward).to(device)
            masks = torch.Tensor(transition.mask).to(device)
            local = self.zipStack(transition.local)
            next_local = self.zipStack(transition.next_local)

            actions_next = []
            for s in next_local:
                next_policy = self.actor(s)
                next_action = self.choose(next_policy)
                actions_next.append(torch.Tensor(next_action))

            '''# Critic Update
            ID = torch.Tensor(states.size()[0], 1).fill_(-1)
            inp = torch.cat((states_next, actions_next[1].unsqueeze(1), ID), dim = 1)
            q_tar = self.target(inp).detach().gather(1, actions_next[0].long().unsqueeze(1))
            q_tar = self.get_lambda_targets(rewards.squeeze(), masks.squeeze(), self.discount, q_tar)
            inp = torch.cat((states, actions[:, 1].unsqueeze(1), ID), dim = 1)
            q = self.critic(inp)
            q = q.gather(1, actions[:, 0].long().unsqueeze(1))
            loss = self.critic.get_loss(q, q_tar)
            self.critic.optimizer.zero_grad()
            loss.backward()
            self.critic.optimizer.step()'''

            ID = torch.Tensor(states.size()[0], 1).fill_(1)
            inp = torch.cat((states_next, actions_next[0].unsqueeze(1), ID), dim = 1)
            q_tar = self.target(inp).detach().gather(1, actions_next[1].long().unsqueeze(1)) # .max(1)? 
            q_tar = self.get_lambda_targets(rewards.squeeze(), masks.squeeze(), self.discount, q_tar)
            inp = torch.cat((states, actions[:, 0].unsqueeze(1), ID), dim = 1)
            q = self.critic(inp)
            q = q.gather(1, actions[:, 1].long().unsqueeze(1))
            loss = self.critic.get_loss(q, q_tar)
            self.critic.optimizer.zero_grad()
            loss.backward()
            self.critic.optimizer.step()

            
            actor_loss = 0
            # Actor Update. Consider doing new_actions
            policies = []
            new_actions = []
            for s in local:
                policy = self.actor(s)
                policies.append(policy)
                new_action = self.choose(policy)
                new_actions.append(torch.Tensor(new_action))

            '''ID = torch.Tensor(states.size()[0], 1).fill_(-1)
            inp = torch.cat((states, new_actions[1].unsqueeze(1), ID), dim = 1)
            q_out = self.critic(inp) #batch x num_actions
            policy = policies[0] #batch x num_actions
            mult = q_out * policy
            baseline = torch.sum(mult, 1).unsqueeze(1)
            q_taken = q_out.gather(1, new_actions[0].long().unsqueeze(1))
            coma = (q_taken - baseline).detach()
            probs_taken = policy.gather(1, new_actions[0].long().unsqueeze(1))
            loss = -(torch.log(probs_taken) * coma).mean()
            actor_loss += loss '''

            ID = torch.Tensor(states.size()[0], 1).fill_(1)
            inp = torch.cat((states, new_actions[0].unsqueeze(1), ID), dim = 1)
            q_out = self.critic(inp) #batch x num_actions
            policy = policies[1] #batch x num_actions
            mult = q_out * policy
            baseline = torch.sum(mult, 1).unsqueeze(1)
            q_taken = q_out.gather(1, new_actions[1].long().unsqueeze(1))
            coma = (q_taken - baseline).detach()
            probs_taken = policy.gather(1, new_actions[1].long().unsqueeze(1))
            loss = -(torch.log(probs_taken) * coma).mean()
            actor_loss += loss 


            self.actorLoss.append(actor_loss)

            if self.homogenous:
                self.actor.optimizer.zero_grad()
                actor_loss.backward()
                nn.utils.clip_grad_norm_(self.actor.parameters(), 1)
                self.actor.optimizer.step()
            else:
                for actor in self.actor:
                    actor.optimizer.zero_grad()
                actor_loss.backward()
                for actor in self.actor:
                    actor.optimizer.step()    

            self.totalSteps += 1
            # self.exp = Memory()
            self.trained = True

            #UPDATE TARGET NETWORK:
            if self.totalSteps % 1 == 0: # THIS IS A TEST
                for target_param, param in zip(self.target.parameters(), self.critic.parameters()):
                    target_param.data.copy_((1 - self.tau) * target_param + self.tau*param.data)

            return 

class CounterActor(nn.Module):
    def __init__(self, params, train):
        super(CounterActor, self).__init__()
        self.h_state_n  = params['h_state_n']
        self.x_state_n  = params['x_state_n']
        self.u_n        = params['u_n']
        self.lr         = train['lr']
        self.mean       = params['mu']
        self.std        = params['std']
        self.eps        = .5

        self.fc1        = nn.Linear(self.x_state_n, self.h_state_n)
        self.gru        = nn.GRU(self.h_state_n, self.h_state_n)
        self.hidden     = nn.Linear(self.h_state_n, self.h_state_n)
        self.fc2        = nn.Linear(self.h_state_n, self.u_n)
        self.soft       = nn.Softmax(dim = 1)
        self.optimizer  = optim.Adam(super(CounterActor, self).parameters(), lr=self.lr)

    def preprocess(self, inputs):
        return (inputs - self.mean) / self.std

    def forward(self, x):
        # x = torch.cat((x, torch.Tensor([ID]).view((1,1)), torch.Tensor(prev_a).unsqueeze(0)), dim=1)
        x = self.preprocess(x)
        x = F.leaky_relu(self.fc1(x))
        x = F.leaky_relu(self.hidden(x))
        out = self.fc2(x)
        policy = self.soft(out)
        policy = (1 - self.eps) * policy + self.eps / self.u_n
        return policy
    

    
    


    
