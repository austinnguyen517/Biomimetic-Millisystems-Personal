#! /usr/bin/env python

import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt 
import torch.optim as optim
from collections import OrderedDict
from network import Network 

class DualNetwork(Network):
    def __init__(self, netParams, trainParams):
        super(DualNetwork, self).__init__(netParams, trainParams)

    def createFeatures(self):
        layers = []
        layers.append(('input_lin', nn.Linear(
                self.in_n, self.hidden_w)))       # input layer
        layers.append(('input_act', self.act))
        layers.append(('input_dropout', nn.Dropout(p = self.d)))
        for d in range(self.depth):
            layers.append(('lin_'+str(d), nn.Linear(self.hidden_w, self.hidden_w)))
            layers.append(('act_'+str(d), self.act))
            layers.append(('dropout_' + str(d), nn.Dropout(p = self.d)))
        self.features = nn.Sequential(OrderedDict(layers))
        layers = []
        for d in range(self.depth):
            layers.append(('val_lin_'+str(d), nn.Linear(self.hidden_w, self.hidden_w)))
            layers.append(('val_act_'+str(d), self.act))
            layers.append(('val_dropout_' + str(d), nn.Dropout(p = self.d)))
        layers.append(('val_out_lin', nn.Linear(self.hidden_w, self.out_n)))
        self.valueStream = nn.Sequential(OrderedDict(layers))
        for d in range(self.depth):
            layers.append(('val_lin_'+str(d), nn.Linear(self.hidden_w, self.hidden_w)))
            layers.append(('val_act_'+str(d), self.act))
            layers.append(('val_dropout_' + str(d), nn.Dropout(p = self.d)))
        layers.append(('val_out_lin', nn.Linear(self.hidden_w, self.out_n)))
        self.advantageStream = nn.Sequential(OrderedDict(layers))

    def forward(self, inputs):
        inputs = torch.FloatTensor(inputs)
        if self.pre:
            inputs = self.preProcessIn(inputs)
        feats = self.features(inputs)
        values = self.valueStream(feats)
        advantages = self.advantageStream(feats)
        outputs = values + (advantages - advantages.mean())
        if self.pre:
            outputs = self.postProcess(outputs)
        return outputs
    