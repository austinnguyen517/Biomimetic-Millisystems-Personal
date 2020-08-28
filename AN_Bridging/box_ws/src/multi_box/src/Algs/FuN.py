#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import torch.optim as optim
import math 
import rospy
import torch.nn.functional as F
from torch.distributions import Categorical
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt

from agent import Agent
from Networks.network import Network
from Networks.feudalNetwork import FeudalNetwork
from Buffers.FeudalBuffer import Memory
from collections import namedtuple

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


Temp = namedtuple('Temp',('goal', 'policy', 'm_value', 'w_value_ext', 'w_value_int', 'm_state'))

class Feudal(object):
    def __init__(self, params, name, task):
        self.name           = name
        self.task           = task
        self.vTrain         = params['train']
        self.agents         = params['agents']
        self.fun            = params['fun']

        self.pubs = {}
        for key in self.agents.keys():
            bot             = self.agents[key]
            self.pubs[key]  = rospy.Publisher(bot['pub'], Vector3, queue_size = 1)
        rospy.Subscriber("/finished", Int8, self.receiveDone, queue_size = 1)

        self.actions        = self.fun['u']
        self.horizon        = self.fun['c']
        self.k              = self.fun['k']
        self.state          = self.fun['s']
        self.d              = self.fun['d']
        self.valueLoss      = []

        self.net            = FeudalNetwork(self.actions, self.state, self.horizon, self.k, self.d).to(device)

        self.m_discount     = self.vTrain['m_gamma']
        self.w_discount     = self.vTrain['w_gamma']
        self.lr             = self.vTrain['lr']
        self.trainMode      = self.vTrain['trainMode']
        self.clip_grad_norm = self.vTrain['clip_grad']
        self.step           = self.vTrain['step']
        self.alpha          = self.vTrain['alpha']
        self.stop           = False

        self.exp            = Memory()
        self.optimizer      = optim.Adam(self.net.parameters(), lr = self.lr)
        self.temp           = None
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

    def get_action(self, s, s_w = None):
        net_out = self.net(torch.FloatTensor(s), self.m_lstm, self.w_lstm, self.goals_horizon)
        policy, goal, self.goals_horizon, self.m_lstm, self.w_lstm, m_value, w_value_ext, w_value_int, m_state = net_out
        self.temp_second = self.temp_first
        self.temp_first = Temp(goal, policy, m_value, w_value_ext, w_value_int, m_state)
        choice = np.asscalar(self.choose(policy))
        action = self.actionMap[choice]
        return choice #single env

    def choose(self, policies):
        m = Categorical(policies)
        action = m.sample()
        action = action.data.cpu().numpy()
        return action
    
    def saveModel(self):
        pass

    def store(self, s, a, r, sprime, aprime, done):
        self.exp.push(s, a, sum(r), 1 - done, self.temp_second.goal, self.temp_second.policy, self.temp_second.m_value, 
                    self.temp_second.w_value_ext, self.temp_second.w_value_int, self.temp_second.m_state)

    def reset(self):
        m_hx                = torch.zeros(1, self.d).to(device)
        m_cx                = torch.zeros(1, self.d).to(device)
        self.m_lstm         = (m_hx, m_cx)

        w_hx                = torch.zeros(1, self.actions * self.k).to(device)
        w_cx                = torch.zeros(1, self.actions * self.k).to(device)
        self.w_lstm         = (w_hx, w_cx)

        self.goals_horizon  = torch.zeros(1, self.horizon + 1, self.d).to(device)
        self.temp_first, self.temp_second = (None, None)
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

    def get_returns(self, rewards, masks, gamma, values):
        returns = torch.zeros_like(rewards)
        running_returns = values[-1].squeeze()

        for t in reversed(range(0, len(rewards)-1)):
            running_returns = rewards[t] + gamma * running_returns * masks[t]
            returns[t] = running_returns

        if returns.std() != 0:
            returns = (returns - returns.mean()) / returns.std()

        return returns

    def train(self):
        if len(self.exp) > self.step:
            transition = self.exp.sample()
            actions = torch.Tensor(transition.action).long().to(device)
            rewards = torch.Tensor(transition.reward).to(device)
            masks = torch.Tensor(transition.mask).to(device)
            goals = torch.stack(transition.goal).to(device)
            policies = torch.stack(transition.policy).to(device)
            m_states = torch.stack(transition.m_state).to(device)
            m_values = torch.stack(transition.m_value).to(device)
            w_values_ext = torch.stack(transition.w_value_ext).to(device)
            w_values_int = torch.stack(transition.w_value_int).to(device)
            
            m_returns = self.get_returns(rewards, masks, self.m_discount, m_values)
            w_returns = self.get_returns(rewards, masks, self.w_discount, w_values_ext)

            intrinsic_rewards = torch.zeros_like(rewards).to(device)
            
            # todo: how to get intrinsic reward before 10 steps
            for i in range(self.horizon, len(rewards)):
                cos_sum = 0
                for j in range(1, self.horizon + 1):
                    alpha = m_states[i] - m_states[i - j]
                    beta = goals[i - j]
                    cosine_sim = F.cosine_similarity(alpha, beta)
                    cos_sum = cos_sum + cosine_sim
                intrinsic_reward = cos_sum / self.horizon
                intrinsic_rewards[i] = intrinsic_reward.detach()
            returns_int = self.get_returns(intrinsic_rewards, masks, self.w_discount, w_values_int)

            m_loss = torch.zeros_like(w_returns).to(device)
            w_loss = torch.zeros_like(m_returns).to(device)

            for i in range(0, len(rewards)-self.horizon):
                m_advantage = m_returns[i] - m_values[i].squeeze(-1)
                alpha = m_states[i + self.horizon] - m_states[i]
                beta = goals[i]
                cosine_sim = F.cosine_similarity(alpha.detach(), beta)
                m_loss[i] = - m_advantage * cosine_sim

                log_policy = torch.log(policies[i] + 1e-5)
                w_advantage = w_returns[i] + returns_int[i] - w_values_ext[i].squeeze(-1) - w_values_int[i].squeeze(-1)
                log_policy = log_policy.gather(-1, actions[i].unsqueeze(-1).unsqueeze(-1))
                w_loss[i] = - (w_advantage) * log_policy.squeeze(-1)
            
            m_loss = m_loss.mean()
            w_loss = w_loss.mean()
            m_loss_value = F.mse_loss(m_values.view(m_returns.size()), m_returns.detach())
            w_loss_value_ext = F.mse_loss(w_values_ext.view(w_returns.size()), w_returns.detach())
            w_loss_value_int = F.mse_loss(w_values_int.view(w_returns.size()), returns_int.detach())

            loss = w_loss + w_loss_value_ext + w_loss_value_int + m_loss + m_loss_value

            self.optimizer.zero_grad()
            loss.backward(retain_graph=False) # TODO: This was changed? Maybe back to true
            torch.nn.utils.clip_grad_norm_(self.net.parameters(), self.clip_grad_norm)
            self.optimizer.step()

            m_hx, m_cx = self.m_lstm
            self.m_lstm = (m_hx.detach(), m_cx.detach())
            w_hx, w_cx = self.w_lstm
            self.w_lstm = (w_hx.detach(), w_cx.detach())
            self.goals_horizon = self.goals_horizon.detach()
            self.exp = Memory()
            self.totalSteps += 1

            self.valueLoss.append(loss)

            return loss


    
