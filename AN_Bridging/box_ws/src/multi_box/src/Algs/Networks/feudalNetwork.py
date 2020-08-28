#! /usr/bin/env python

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.nn.init as init
from collections import OrderedDict
from network import Network
import numpy as np


class FeudalNetwork(nn.Module):
    def __init__(self, num_actions, num_state, horizon, k, d):
        super(FeudalNetwork, self).__init__()
        self.manager = Manager(num_actions, num_state, d)
        self.worker = Worker(num_actions, num_state, k, d)
        self.horizon = horizon

    def forward(self, x, m_lstm, w_lstm, goals_horizon):
        m_inputs = (x, m_lstm)
        goal, m_lstm, m_value, m_state = self.manager(m_inputs)
        
        #Assumption: our starting goals summed together must be 0
        goals_horizon = torch.cat([goals_horizon[:, 1:], goal.unsqueeze(1)], dim=1)
        
        w_inputs = (x, w_lstm, goals_horizon)
        policy, w_lstm, w_value_ext, w_value_int = self.worker(w_inputs)
        return policy, goal, goals_horizon, m_lstm, w_lstm, m_value, w_value_ext, w_value_int, m_state

######################################################################################################

class Manager(nn.Module):
    def __init__(self, num_actions, num_state, d):
        super(Manager, self).__init__()
        self.fc = nn.Linear(num_state, d)

        self.lstm = nn.LSTMCell(d, hidden_size= d)

        self.lstm.bias_ih.data.fill_(0)
        self.lstm.bias_hh.data.fill_(0)

        self.fc_critic1 = nn.Linear(d, 256)
        self.fc_critic2 = nn.Linear(256, 1)

    def forward(self, inputs):
        x, (hx, cx) = inputs
        x = F.relu(self.fc(x))
        state = x
        hx, cx = self.lstm(x, (hx, cx))

        goal = cx
        value = F.relu(self.fc_critic1(goal))
        value = self.fc_critic2(value)

        i = np.random.random()
        if i < .05: #generate random goal
            num = goal.numel()
            goal = torch.empty(num).normal_().view(goal.size())
        
        goal_norm = torch.norm(goal, p=2, dim=1).unsqueeze(1)
        goal = goal / goal_norm.detach()
        return goal, (hx, cx), value, state

######################################################################################################
    
class Worker(nn.Module):
    def __init__(self, num_actions, num_state, k, d):
        self.num_actions = num_actions
        super(Worker, self).__init__()
        self.k = k

        self.percep = nn.Linear(num_state, d)

        self.lstm = nn.LSTMCell(d, hidden_size=num_actions * k)
        self.lstm.bias_ih.data.fill_(0)
        self.lstm.bias_hh.data.fill_(0)

        self.fc = nn.Linear(d, k, bias=False)

        self.fc_critic1 = nn.Linear(num_actions * k, num_actions * k)
        self.fc_critic1_out = nn.Linear(num_actions * k, 1)
        
        self.fc_critic2 = nn.Linear(num_actions * k, num_actions * k)
        self.fc_critic2_out = nn.Linear(num_actions * k, 1)
        
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')

    def forward(self, inputs):
        x, (hx, cx), goals = inputs
        x = self.percep(x)
        hx, cx = self.lstm(x, (hx, cx))

        value_ext = F.relu(self.fc_critic1(hx))
        value_ext = self.fc_critic1_out(value_ext)
        
        value_int = F.relu(self.fc_critic2(hx))
        value_int = self.fc_critic2_out(value_int)

        worker_embed = hx.view(hx.size(0), self.num_actions, self.k)
        
        goals = goals.sum(dim=1)
        goal_embed = self.fc(goals.detach())
        goal_embed = goal_embed.unsqueeze(-1)

        policy = torch.bmm(worker_embed, goal_embed)
        policy = policy.squeeze(-1)  #working with one env
        policy = F.softmax(policy, dim=-1)
        return policy, (hx, cx), value_ext, value_int
