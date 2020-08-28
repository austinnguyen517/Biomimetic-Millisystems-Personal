#! /usr/bin/env python

import numpy as np 
import torch 
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from network import Network

class AutoEncoder(Network):
    def __init__(self, netParams, trainParams):
        super(AutoEncoder, self).__init__(netParams, trainParams)


    def forward(self, x):
        x = self.preprocess(x)
        for i in range(self.n_layers):
            x = eval((self.act[i] + '(self.fc{}(x))').format(i+1))
        x = eval(self.act[self.n_layers] + '(self.output(x))')
        return self.postprocess(x)

    def encode(self, x):
        x = self.preprocess(x)
        for i in range(self.n_layers):
            x = eval((self.act[i] + '(self.fc{}(x))').format(i+1))
        return x.detach()
    
    def decode(self, x):
        x = self.output(x)
        x = eval(self.act[self.n_layers] + '(x)')
        return self.postprocess(x)
    
    def postprocess(self, x):
        return (x * self.std) + self.mean
    
    def train(self, x):
        outputs = self.forward(x)
        criterion = nn.MSELoss()
        loss = criterion(outputs, x)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        print('AutoEncoder Loss: ', loss.detach())
