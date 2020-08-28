#! /usr/bin/env python

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from collections import OrderedDict
import numpy as np

class Network(nn.Module):
    def __init__(self, netParams, trainParams):
        super(Network,self).__init__()
        self.layers = []
        self.neurons = netParams['neurons']
        self.n_layers = len(self.neurons) - 2
        self.act = netParams['act']
        self.lr = trainParams['lr']
        self.std = netParams['std']
        self.mean = netParams['mu']

        self.createFeatures()

        self.optimizer =  optim.Adam(super(Network, self).parameters(), lr=self.lr)

    def preprocess(self, inputs):
        return (inputs - self.mean) / self.std

    
    def createFeatures(self):
        for i, (fan_in, fan_out) in enumerate(zip(self.neurons[:-2], self.neurons[1:-1])):
            layer = nn.Linear(fan_in, fan_out)
            torch.nn.init.uniform_(layer.weight, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            torch.nn.init.uniform_(layer.bias, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            exec('self.fc{} = layer'.format(i+1))

        layer = nn.Linear(self.neurons[-2], self.neurons[-1])
        torch.nn.init.uniform_(layer.weight, -3e-3, 3e-3)
        torch.nn.init.uniform_(layer.bias, -3e-3, 3e-3)
        self.output = layer


    def forward(self, inputs):
        x = self.preprocess(inputs)
        for i in range(self.n_layers):
            x = eval((self.act[i] + '(self.fc{}(x))').format(i+1))

        return self.output(x)


    def get_loss(self, val, next_val):
        criterion = nn.MSELoss()
        return criterion(val, next_val)

