#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import math 


class Model(nn.Module):
    def __init__(self, net_params, train_params):
        super(Model,self).__init__()
        self.layers = []
        self.neurons = net_params['neurons']
        self.n_layers = len(self.neurons) - 2
        self.act = net_params['act']
        self.lr = train_params['lr']
        self.std = net_params['std']
        self.mean = net_params['mu']

        self.max_size = train_params['buffer']
        self.batch_size = train_params['batch']
        self.noise = train_params['noise']
        self.temperature = train_params['certainty_temp']
        self.num_actions = 5
        self.createFeatures()

        self.transitions = np.zeros((1, 19))
        self.size = 1

        self.optimizer =  optim.Adam(super(Model, self).parameters(), lr=self.lr)

    def preprocess(self, inputs):
        return (inputs - self.mean) / self.std

    
    def createFeatures(self):
        for i, (fan_in, fan_out) in enumerate(zip(self.neurons[:-2], self.neurons[1:-1])):
            layer = nn.Linear(fan_in, fan_out)
            torch.nn.init.uniform_(layer.weight, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            torch.nn.init.uniform_(layer.bias, -1./np.sqrt(fan_in), 1./np.sqrt(fan_in))
            exec('self.fc{} = layer'.format(i+1))
            layer = nn.Dropout(p=.2)
            exec('self.drop{} = layer'.format(i+1))

        self.mean_layer = nn.Linear(self.neurons[-2], self.neurons[-1])
        torch.nn.init.uniform_(self.mean_layer.weight, -3e-3, 3e-3)
        torch.nn.init.uniform_(self.mean_layer.bias, -3e-3, 3e-3)


    def forward(self, inputs):
        x = self.preprocess(inputs)
        for i in range(self.n_layers):
            x = eval((self.act[i] + '(self.fc{}(x))').format(i+1))
            x = eval(('self.drop{}(x)').format(i+1))
        
        x = self.mean_layer(x)
        mean = x.narrow(1, 0, 9)
        std = x.narrow(1, 9, 1)
        return mean, std

    def saveModel(self):
        torch.save(self.valueNet.state_dict(), '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/GP_model.txt')
        pass
    
    def change_to_evaluation_mode(self):
        for i in range(len(self.neurons[:-2])):
            exec('self.drop{}.eval()'.format(i+1))
    
    def reset_buffer(self):
        self.transitions = np.zeros((1, neurons[0] * 2 + 1))
        self.size = 1

    def predict(self, feature, primitive_index):
        one_hot = self.one_hot(np.array([primitive_index]))
        feature = torch.FloatTensor(np.hstack((feature, one_hot)))
        mu, log_std = self.forward(feature)
        std = log_std.exp()
        return None, mu.detach().numpy(), std.detach().numpy()
    
    def store(self, feature, primitive_index, new_feature):
        print(feature.shape, primitive_index.shape, new_feature.shape)
        self.transitions = np.vstack((self.transitions, np.hstack((feature, primitive_index.reshape(-1, 1), new_feature))))
        self.size += feature.shape[0]
        if self.size > self.max_size:
            self.transitions = self.transitions[1:, :]
    
    def one_hot(self, actions):
        b = np.zeros((actions.size, self.num_actions))
        b[np.arange(actions.size),actions.astype(int)] = 1
        return b

    def train(self):
        # probabilistic gaussian for new feature predictor
        if self.size > 500:
            state_n = 9
            indices = np.random.choice(self.size - 1, self.batch_size) + 1
            batch = self.transitions[indices]

            feature_in = batch[:, :state_n] 
            actions = batch[:, state_n]
            feature_out = batch[:, -state_n:]
            feature_in = feature_in + + np.random.normal(scale=self.noise, size=feature_in.shape) # noise

            one_hot_encoding = self.one_hot(actions.ravel())
            feature_in = torch.FloatTensor(np.hstack((feature_in, one_hot_encoding)))

            mu, log_std = self.forward(feature_in)
            std = log_std.exp()
            square_delta = torch.pow(mu - torch.FloatTensor(feature_out), 2)
            pdf_loss = -torch.log((-square_delta / (2 * torch.pow(std, 2))).exp() * (1/std) + 1e-20) + std * 15
            pdf_loss = pdf_loss.mean()

            mean_loss = square_delta.mean()

            pdf_loss = (self.temperature) * mean_loss + (1 - self.temperature) * pdf_loss

            self.optimizer.zero_grad()
            pdf_loss.backward()
            self.optimizer.step()
            return pdf_loss
