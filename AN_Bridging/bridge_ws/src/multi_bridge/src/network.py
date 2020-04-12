#! /usr/bin/env python

import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt 
import torch.optim as optim
from collections import OrderedDict
from sklearn.preprocessing import StandardScaler, MinMaxScaler, RobustScaler, QuantileTransformer

class Network(nn.Module):
    def __init__(self, netParams, trainParams):
        super(Network,self).__init__()
        self.in_n = netParams['in_n']
        self.out_n = netParams['out_n']
        self.prob = netParams['prob']
        self.hidden_w = netParams['hidden']
        self.depth = netParams['depth']
        self.act = netParams['act']
        self.d = netParams['dropout']
        self.lr = trainParams['lr']
        self.pre = netParams['preprocess']
        loss = netParams['loss_fnc']
        self.sigmoid = nn.Sigmoid()

        if self.prob:
            self.out_n *= 2

        if loss == "policy_gradient":
            self.loss_fnc = None
        elif loss == "MSE":
            self.loss_fnc = nn.MSELoss()

        if self.pre:
            self.scalarInput = StandardScaler() #or any of the other scalers...look into them 
            self.scalarOutput = StandardScaler()

        layers = []
        layers.append(('dynm_input_lin', nn.Linear(
                self.in_n, self.hidden_w)))       # input layer
        layers.append(('dynm_input_act', self.act))
        layers.append(('dynm_input_dropout', nn.Dropout(p = self.d)))
        for d in range(self.depth):
            layers.append(('dynm_lin_'+str(d), nn.Linear(self.hidden_w, self.hidden_w)))
            layers.append(('dynm_act_'+str(d), self.act))
            layers.append(('dynm_dropout_' + str(d), nn.Dropout(p = self.d)))
        layers.append(('dynm_out_lin', nn.Linear(self.hidden_w, self.out_n)))
        self.features = nn.Sequential(OrderedDict(layers))

        self.optimizer =  optim.Adam(super(Network, self).parameters(), lr=self.lr)
    
    def preProcessIn(self, inputs):
        if self.pre:
            self.scalarInput.fit(inputs)
            norm = self.scalarInput.transform(inputs)
            return norm
        return inputs

    def postProcess(self, outputs):
        #depeneds what we are trying to do 
        if self.pre:
            return self.scalarOutput.inverse_transform(outputs)
        return outputs

    def forward(self, inputs):
        if self.pre:
            inputs = self.preProcessIn(inputs)
        outputs = self.features(inputs)
        if self.pre:
            outputs = self.postProcess(outputs)
        return outputs
    

    def predict(self, input):
        if self.pre:
            input = self.preProcessIn(input)
        input = torch.FloatTensor(input)
        x = self.features(input)
        if self.pre:
            x = self.postProcess(x)
        '''NO ACCOUNT FOR SELF.PROB'''
        return x


    def train_cust(self, inputs, outputs, advantages = None):
        self.train()
        self.optimizer.zero_grad()
        lossTot = 0
        for i in range(self.epochs):
            out = self.predict(inputs)
            if self.loss_fnc != None: #MSELoss
                assert advantages == None 
                loss = self.loss_fnc(out, outputs)
            else: #policy gradient
                #each row is a sample. Outputs represent our actions!
                means = out[:, :int(self.out_n/2)]
                if self.prob:
                    std = out[:, int(self.out_n/2):]
                    outputs = torch.FloatTensor(outputs)
                    prob = torch.exp(-(1/2)*(((means - outputs) ** 2 )/  std))
                    prob = (1/((2*np.pi)**(1/2) * std) * prob)
                    loss = -torch.sum(torch.log(prob)*advantages)
                else:
                    loss = (torch.sum(means * advantages))*(1 / (means.size())[0])
            loss.backward() 
            self.optimizer.step()
            lossTot += loss
        return lossTot

