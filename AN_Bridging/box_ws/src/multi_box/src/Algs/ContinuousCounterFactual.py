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
from Networks.softNetwork import SoftNetwork
from Buffers.CounterFactualBuffer import Memory
from collections import namedtuple
from torch.distributions import Normal

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
Temp = namedtuple('Temp',('log_prob', 'means', 'stds'))

class CounterContinuous(object):
    def __init__(self, params, name, task):
        self.name           = name
        self.task           = task
        self.vTrain         = params['valTrain']
        self.vPars          = params['valPars']
        self.aTrain         = params['actTrain']
        self.aPars          = params['actPars']
        self.agents         = params['agents']

        self.pubs = {}
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
            self.actor      = SoftNetwork(self.aPars, self.aTrain).to(device) 
        else:
            self.actor      = [SoftNetwork(self.aPars, self.aTrain) for i in range(len(self.agents))]

        for target_param, param in zip(self.target.parameters(), self.critic.parameters()):
            target_param.data.copy_(param.data)

        self.clip_grad_norm = self.aTrain['clip']
        self.trainMode      = self.vPars['trainMode']
        self.batch_size     = self.vTrain['batch_size']
        self.discount       = self.vTrain['gamma']
        self.range          = self.aPars['mean_range']
        self.td_lambda      = .8
        self.tau            = .005
        self.lower_bound    = self.aTrain['clamp'][2]
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
            a1, log_prob1, z, mu1, log_std1 = self.actor(torch.FloatTensor(s_split[0]))
            a2, log_prob2, z, mu2, log_std2 = self.actor(torch.FloatTensor(s_split[1]))
        else: # TODO: Fix this below:
            a1, h_new1, log_prob1, mu1, std1 = self.actor[0](torch.FloatTensor(s_split[0]), self.h[0])
            a2, h_new2, log_prob2, mu2, std2 = self.actor[1](torch.FloatTensor(s_split[1]), self.h[1])
        action = [a1.detach().numpy().ravel(), a2.detach().numpy().ravel()]
        return [a1, a2]

    def choose(self, policies):
        m = Categorical(policies)
        action = m.sample()
        action = action.data.cpu().numpy()
        return np.asscalar(action)

    
    def saveModel(self):
        pass

    def store(self, s, a, r, sprime, aprime, done, local, next_local):
        self.exp.push(s, a, r, 1 - done, aprime, sprime, local, next_local)

    def reset(self):
        curr = self.actor.clamp[0]
        if self.trained:
            new = max(self.lower_bound, .05 * self.lower_bound + .95*curr)
            self.actor.clamp = (new, self.actor.clamp[1], self.lower_bound)
        self.trained = False
        return 

    def get_grad_norm(self, model):
        total_norm = 0
        for p in model.parameters():
            if p.grad is None:
                continue
            param_norm = p.grad.data.norm(2)
            total_norm += param_norm.item() ** 2
        grad_norm = total_norm ** (1. / 2)
        return grad_norm

    def get_lambda_targets(self, rewards, mask, gamma, target_qs):
        target_qs = target_qs.squeeze()
        ret = target_qs.new_zeros(*target_qs.shape)
        ret[-1] = target_qs[-1] * mask[-1]


        for t in range(ret.size()[0] - 2, -1,  -1): 
            ret[t] = self.td_lambda * gamma * ret[t + 1] + \
                mask[t] * (rewards[t] + (1 - self.td_lambda) * gamma * target_qs[t + 1])
        return ret.unsqueeze(1)

    def zipStack(self, data):
        data        = zip(*data)
        data        = [torch.stack(d).squeeze().to(device) for d in data]
        return data

    def monte_carlo(self, mean, std, n = 500):
        # returns tensors representing n sampled from mean and std
        normal = Normal(mean, std)
        return normal.sample((n,))


    def train(self, episode_done = False):
        if len(self.exp) >= 500:

            transition = self.exp.sample(self.batch_size)
            states = torch.squeeze(torch.Tensor(transition.state)).to(device)
            actions = self.zipStack(transition.action)
            rewards = torch.Tensor(transition.reward).to(device)
            states_next = torch.squeeze(torch.Tensor(transition.next_state)).to(device) 
            masks = torch.Tensor(transition.mask).to(device)
            local = self.zipStack(transition.local)
            next_local = self.zipStack(transition.next_local)

            actions_next = []
            for s in next_local:
                a, log_prob, _, _, _ = self.actor(s)
                actions_next.append(a.detach())
            inp = torch.cat((states_next, actions_next[0], actions_next[1]), dim = 1)
            q_tar = rewards.unsqueeze(1) + self.discount * masks.unsqueeze(1) * self.target(inp)
            inp = torch.cat((states, actions[0].detach(), actions[1].detach()), dim = 1)
            q = self.critic(inp)
            loss = self.critic.get_loss(q, q_tar.detach())
            self.critic.optimizer.zero_grad()
            loss.backward()
            self.critic.optimizer.step()
            self.valueLoss.append(loss)

            
            actor_loss = 0
            actions = []
            means = []
            log_stds = []
            log_probs = []
            for s in local:
                a, log_prob, z, mu, log_std = self.actor(s)
                actions.append(a)
                means.append(mu)
                log_stds.append(log_std)
                log_probs.append(log_prob)

            # train first agent 
            inp = torch.cat((states, actions[0], actions[1].detach()), dim = 1)
            q_out = self.critic(inp)
            samples = self.monte_carlo(means[0], log_stds[0].exp())
            samples = self.range * torch.tanh(samples)
            repeat_s = states.unsqueeze(0)
            repeat_s = repeat_s.expand(samples.size()[0], repeat_s.size()[1], repeat_s.size()[2]) 
            repeat_a = actions[1].unsqueeze(0)
            repeat_a = repeat_a.expand(samples.size()[0], repeat_a.size()[1], repeat_a.size()[2])
            inp = torch.cat((repeat_s, samples, repeat_a), dim = 2)
            baseline = self.critic(inp).mean(0)
            coma = (q_out - baseline).detach()  
            actor_loss -= (log_probs[0].view(coma.size())*(coma)).mean()

            # train second agent
            inp = torch.cat((states, actions[0].detach(), actions[1]), dim = 1)
            q_out = self.critic(inp)
            samples = self.monte_carlo(means[1], log_stds[1].exp())
            samples = self.range * torch.tanh(samples)
            repeat_a = actions[0].unsqueeze(0)
            repeat_a = repeat_a.expand(samples.size()[0], repeat_a.size()[1], repeat_a.size()[2])
            inp = torch.cat((repeat_s, repeat_a, samples), dim = 2)
            baseline = self.critic(inp).mean(0)
            coma = (q_out - baseline).detach()
            actor_loss -= (log_probs[1].view(coma.size()) * (coma)).mean()

            if self.homogenous:
                self.actor.optimizer.zero_grad()
                actor_loss.backward()
                self.actor.optimizer.step()
            else:
                for actor in self.actor:
                    actor.optimizer.zero_grad()
                actor_loss.backward()
                for actor in self.actor:
                    torch.nn.utils.clip_grad_norm_(actor.parameters(), self.clip_grad_norm)
                    actor.optimizer.step()    
            self.totalSteps += 1
            self.trained = True

            #UPDATE TARGET NETWORK:
            if self.totalSteps % 50 == 0:
                for target_param, param in zip(self.target.parameters(), self.critic.parameters()):
                    target_param.data.copy_(param.data)
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
        self.range      = params['mean_range']

        self.fc1        = nn.Linear(self.x_state_n, self.h_state_n)
        self.gru        = nn.GRU(self.h_state_n, self.h_state_n, num_layers = 1, batch_first = True)
        self.mean_fc    = nn.Linear(self.h_state_n, 2)
        self.std_fc     = nn.Linear(self.h_state_n, 2)
        self.optimizer  = optim.Adam(super(CounterActor, self).parameters(), lr=self.lr)

    def preprocess(self, inputs):
        return (inputs - self.mean) / self.std

    def forward(self, x, h):
        x = self.preprocess(x)
        inp = self.fc1(x)

        inp = torch.unsqueeze(inp, 0)

        out, h_new = self.gru(inp, h)
        out_n = out.size()
        out = out.view(out_n[0], out_n[2])
        
        mean = self.mean_fc(out)
        std = self.std_fc(out)
        print(std)

        normal = Normal(mean, std)
        
        z = normal.sample()
        action = self.range * torch.tanh(z)
        log_prob = normal.log_prob(z) - torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(-1, keepdim=True)

        return action.squeeze(), h_new, log_prob, mean, std
    

    
    


    
