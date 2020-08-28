#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import math 
import sys


class Model(nn.Module):
    def __init__(self, net_params, train_params, task):
        super(Model,self).__init__()
        self.layers = []
        self.neurons = net_params['neurons']
        self.dropout = net_params['dropout']
        self.n_layers = len(self.neurons) - 2
        self.act = net_params['act']
        self.lr = train_params['lr']
        self.std = net_params['std']
        self.mean = net_params['mu']

        self.max_size = train_params['buffer']
        self.batch_size = train_params['batch']
        self.noise = train_params['noise']
        self.num_actions = net_params['u_n']
        self.one_hot_flag = train_params['one_hot']
        self.createFeatures()

        self.transitions = np.zeros((1, 20))
        self.size = 1

        self.optimizer =  optim.Adam(super(Model, self).parameters(), lr=self.lr, weight_decay = 0.01)

    def preprocess(self, inputs):
        return (inputs - self.mean) / self.std

    
    def createFeatures(self):
        for i, (fan_in, fan_out) in enumerate(zip(self.neurons[:-2], self.neurons[1:-1])):
            layer = nn.Linear(fan_in, fan_out)
            torch.nn.init.uniform_(layer.weight, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            torch.nn.init.uniform_(layer.bias, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            exec('self.fc{} = layer'.format(i+1))
            layer = nn.Dropout(p=self.dropout)
            exec('self.drop{} = layer'.format(i+1))

        self.output_layer = nn.Linear(self.neurons[-2], self.neurons[-1])
        torch.nn.init.uniform_(self.output_layer.weight, -3e-3, 3e-3)
        torch.nn.init.uniform_(self.output_layer.bias, -3e-3, 3e-3)


    def forward(self, inputs):
        x = self.preprocess(inputs)
        for i in range(self.n_layers):
            x = eval((self.act[i] + '(self.fc{}(x))').format(i+1))
            x = eval(('self.drop{}(x)').format(i+1))
        
        x = self.output_layer(x)
        # mean = x.narrow(1, 0, 9)
        # std = x.narrow(1, 9, 1)
        return x
    
    def change_to_evaluation_mode(self):
        for i in range(len(self.neurons[:-2])):
            exec('self.drop{}.eval()'.format(i+1))
    
    def reset_buffer(self):
        self.transitions = np.zeros((1, neurons[0] * 2 + 1))
        self.size = 1

    def predict(self, feature, action):
        if self.one_hot_flag:
            one_hot = self.one_hot(np.array(action))
            inpt = torch.FloatTensor(np.hstack((feature, one_hot)))
        else:
            inpt = torch.FloatTensor(np.hstack((feature, action)))
        change_in_feature = self.forward(inpt)
        return torch.FloatTensor(feature[:, :self.neurons[-1]]) + change_in_feature
        '''
        mu, log_std = self.forward(feature)
        std = log_std.exp()
        return None, mu.detach().numpy(), std.detach().numpy()'''
    
    def store(self, feature, action, new_feature):
        self.transitions = np.vstack((self.transitions, np.hstack((feature, action.reshape(feature.shape[0], -1), new_feature))))
        self.size += feature.shape[0]
        if self.size > self.max_size:
            self.transitions = self.transitions[1:, :]
            self.size -= 1
    
    def one_hot(self, actions):
        b = np.zeros((actions.size, self.num_actions))
        b[np.arange(actions.size),actions.astype(int)] = 1
        return b

    def train(self):
        # probabilistic gaussian for new feature predictor
        indices = np.random.choice(self.size - 1, self.batch_size) + 1 # NOTE: We don't include the first index
        batch = self.transitions[indices]

        if self.one_hot_flag:
            feature_in = batch[:, :self.neurons[-1] + 1] # plus identifier 
            actions = batch[:, -self.neurons[-1] - 1]
            feature_out = batch[:, -self.neurons[-1]:]
            delta_feature = feature_out - feature_in[:, :self.neurons[-1]]
        else:
            feature_in = batch[:, :10]
            actions = batch[:, 10:12]
            feature_out = batch[:, -9:]
            delta_feature = feature_out - feature_in[:, :feature_out.shape[1]]

        feature_in = feature_in + np.random.normal(scale=self.noise, size=feature_in.shape) # noise

        if self.one_hot_flag:
            one_hot_encoding = self.one_hot(actions.ravel())
            feature_in = torch.FloatTensor(np.hstack((feature_in, one_hot_encoding)))
        else:
            feature_in = torch.FloatTensor(np.hstack((feature_in, actions)))

        mean = self.forward(feature_in)

        square_delta = torch.pow(mean - torch.FloatTensor(delta_feature), 2)
        square_delta = torch.sum(square_delta, axis=1)
        mean_loss = square_delta.mean()

        self.optimizer.zero_grad()
        mean_loss.backward()
        self.optimizer.step()
        return mean_loss
