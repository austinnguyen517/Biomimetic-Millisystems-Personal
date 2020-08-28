#! /usr/bin/env python

import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt 
import torch.optim as optim
from collections import OrderedDict
from network import Network

class A2CNetwork(Network):
    def __init__(self, netParams, trainParams):
        super(A2CNetwork,self).__init__(netParams, trainParams)
        self.mean_range = netParams['mean_range']
        self.logstd_range = netParams['logstd_range']
        self.tan = nn.Tanh()

    def forward(self, inputs):
        inputs = torch.FloatTensor(inputs)
        if self.pre:
            inputs = self.preProcessIn(inputs)
        outputs = self.features(inputs)
        outputs = self.tan(outputs).view(-1, self.out_n)
        mean = self.mean_range*outputs[:, :int(self.out_n/2)]
        log_std = self.logstd_range * outputs[:, int(self.out_n/2):]
        normal = Normal(mean,torch.exp(log_std))
        action = normal.sample()
        log_prob = normal.log_prob(action + 1e-9)
        return action, log_prob

