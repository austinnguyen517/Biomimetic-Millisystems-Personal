#! /usr/bin/env python

import torch
import torch.nn as nn
from torch.distributions import Normal
import torch.optim as optim
import torch.nn.functional as F
import torch.nn.init as init
from collections import OrderedDict
from network import Network
import numpy as np

class SoftNetwork(Network):
    def __init__(self, netParams, trainParams):
        super(SoftNetwork,self).__init__(netParams, trainParams)
        self.discrete = netParams['discrete']
        if not self.discrete:
            self.mean_range = netParams['mean_range']
            self.clamp = trainParams['clamp']
        else:
            self.soft = nn.Softmax(dim = 1) # TODO: check this 
    
    def createFeatures(self):
        for i, (fan_in, fan_out) in enumerate(zip(self.neurons[:-2], self.neurons[1:-1])):
            layer = nn.Linear(fan_in, fan_out)
            init.uniform_(layer.weight, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            init.uniform_(layer.bias, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            exec('self.fc{} = layer'.format(i+1))

        self.output = nn.Linear(self.neurons[-2], self.neurons[-1])
        self.output.weight.data.uniform_(-3e-3, 3e-3)
        self.output.bias.data.uniform_(-3e-3, 3e-3)
        
        if not self.discrete:
            self.log_std_output = nn.Linear(self.neurons[-2], self.neurons[-1])
            self.log_std_output.weight.data.uniform_(-3e-3, 3e-3)
            self.log_std_output.bias.data.uniform_(-3e-3, 3e-3)

    def choose(self, policies):
        m = Categorical(policies)
        action = m.sample()
        action = action.data.cpu().numpy()
        return action

    def forward(self, inputs):
        x = self.preprocess(inputs)
        for i in range(self.n_layers):
            x = eval('F.leaky_relu(self.fc{}(x))'.format(i+1))
        mu = self.output(x)
        if self.discrete:
            policy = self.soft(mu)
            action = self.choose(policy)
            log_prob = torch.log(policy.gather(1, action.unsqueeze(1)))
            return action, log_prob, None, None, None
        else:
            log_std = self.log_std_output(x)
            log_std = torch.clamp(log_std, self.clamp[0], self.clamp[1])
            std = torch.exp(log_std)

            normal = Normal(mu, std)
            z = normal.sample()
            action = torch.tanh(z)
            log_prob = normal.log_prob(z) - torch.log(1 - action.pow(2) + 1e-6)
            log_prob = log_prob.sum(-1, keepdim=True)

            return self.mean_range * action, log_prob, z, mu, log_std

    def get_action(self, s):
        s = torch.FloatTensor(s).unsqueeze(0)
        a, _, _, _, _ = self.forward(s)
        return self.mean_range * a.detach().cpu().numpy()[0]
    

