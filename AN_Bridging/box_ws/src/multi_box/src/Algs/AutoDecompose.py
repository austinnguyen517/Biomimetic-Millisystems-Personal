#! /usr/bin/env python

import numpy as np 
import math 
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt
from sklearn.cluster import SpectralClustering
import torch
import torch.nn as nn 
import torch.optim as optim
import torch.nn.functional as F
from torch.nn.utils.rnn import pad_sequence


class Decompose(object):
    def __init__(self, params, name, task):
        self.name = name
        self.task = task
        self.agents = params['agents']
        self.params = params['params']
        rospy.Subscriber("/finished", Int8, self.receiveDone, queue_size = 1)
        self.pubs = {}
        for key in self.agents.keys():
            bot = self.agents[key]
            self.pubs[key] = rospy.Publisher(bot['pub'], Vector3, queue_size = 1)

        self.stop           = False
        self.time           = 1
        self.clusters       = self.params['clusters']
        self.state_n        = self.params['state_n']
        self.mode           = self.params['mode']
        self.horizon        = self.params['horizon']
        self.noise          = self.params['noise']
        self.epochs         = 1000
        self.partitions     = 4
        self.data           = None

        task.initAgent(self)
    
        while(not self.stop):
            x = 1+1
        
        if self.mode == 'Spectral':
            self.spectralCluster()
        elif self.mode == 'RNN':
            self.fitLSTM()

        task.postTraining()
    
    def spectralCluster(self):
        self.clustering = SpectralClustering(n_clusters = self.clusters, random_state = 0).fit(self.data)
        self.labels = self.clustering.labels_
        self.data = self.data[:, :-1] # get rid of time
    
    
    def fitLSTM(self):

        class Network(nn.Module):
            def __init__(self, state, hidden, out, lr, horizon):
                super(Network, self).__init__()
                self.lstm = nn.LSTM(state, hidden_size = hidden)
                self.fc = nn.Linear(hidden, out)
                self.horizon = horizon
                self.optimizer = optim.Adam(super(Network, self).parameters(), lr=lr)
            
            def forward(self, inpt):
                out, h = self.lstm(inpt) # default to zero hidden
                res = out[self.horizon - 1, :, :]
                res = self.fc(res)
                return res 
        
        class Encoder(nn.Module):
            def __init__(self, state, hidden, out, lr):
                super(Encoder, self).__init__()
                self.lstm = nn.LSTM(state, hidden_size = hidden)
                self.optimizer = optim.Adam(super(Encoder, self).parameters(), lr=lr)
            
            def forward(self, inpt):
                out, h = self.lstm(inpt) # default to zero hidden
                return out

        class Decoder(nn.Module):
            def __init__(self, state, hidden, out, lr):
                super(Decoder, self).__init__()
                self.lstm = nn.LSTM(hidden, hidden_size = hidden)
                self.optimizer = optim.Adam(super(Decoder, self).parameters(), lr=lr)
            
            def forward(self, inpt):
                out, h = self.lstm(inpt) # default to zero hidden
                return out
        

        decomposer = Network(self.state_n, 64, self.state_n, 1e-4, self.horizon)
        x_project = np.hstack((np.cos(self.data[:-1, 0:1]), np.sin(self.data[:-1, 0:1])))
        y_project = np.hstack((np.cos(self.data[:-1, 0:1] + np.pi/2), np.sin(self.data[:-1,0:1] + np.pi/2)))

        self.data = self.data[:, 1:]
        deltas = self.data[1:, :self.state_n] - self.data[:-1, :self.state_n]
        x_project = np.sum(deltas[:, :2] * x_project, axis = 1).reshape(-1, 1)
        y_project = np.sum(deltas[:, :2] * y_project, axis = 1).reshape(-1, 1)
        projected = np.hstack((x_project, y_project))
        deltas = np.hstack((projected, deltas[:,2:]))

        length = deltas.shape[0]

        sections_n = length - self.horizon # need to account for the resulting state as well
        groups = np.zeros((sections_n, self.horizon, self.state_n))
        labels = np.zeros((sections_n, self.state_n))
        true_labels = np.zeros((sections_n, self.state_n))
        for i in range(sections_n):
            groups[i] = deltas[i: i+self.horizon, :]
            true_labels[i] = deltas[i+self.horizon, :] # labels for testing
            labels[i] = np.mean(groups[i], axis = 0).reshape(1, -1) # labels for training
        
        data = torch.from_numpy(groups).float()
        data = data.transpose(0, 1)
        labels = torch.from_numpy(labels).float()
        true_labels = torch.from_numpy(true_labels).float()
        size = data.size()

        for j in range(self.epochs):
            noisy_data = data + torch.randn(size[0], size[1], size[2])/5
            res = decomposer(noisy_data) #defaults initial hidden to zero
            criterion = nn.MSELoss()
            loss = criterion(res, labels)
            decomposer.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(decomposer.parameters(), 1)
            decomposer.optimizer.step()
        
        # Getting the errors 
        data = data #+ torch.randn(size[0], size[1], size[2])/5
        res = decomposer(data)
        errors = torch.sum(abs(res - true_labels), dim=1).squeeze().detach().numpy()
        self.labels = np.hstack((np.zeros(self.horizon), errors, np.zeros(1)))
        self.labels = np.where(self.labels <= .05, 0, self.labels) 

        '''hierarchy_indices = (np.argpartition(self.labels, -self.partitions)[-self.partitions:]).tolist()
        deltas = np.vstack((deltas, np.zeros((1, self.state_n))))
        encoder = Encoder(self.state_n, 128, 128, 3e-4)
        decoder = Decoder(self.state_n, 128, 128, 3e-4)
        hierarchy_indices = [0] + hierarchy_indices + [deltas.shape[0]]
        hierarchy_indices.sort()
        groups = []
        for i in range(len(hierarchy_indices) - 1):
            add = torch.from_numpy(deltas[hierarchy_indices[i]: hierarchy_indices[i+1], :])
            groups.append(add)
        sequences = pad_sequence(groups)

        for i in range(1):
            rep = encoder(sequences.double())
            print(rep.size())
            # get last output from encoder. Pass into decoder
            # get loss by subtracting sequences from the output
            # only gather the losses from the sequence lengths
            # backward. step'''


        
        

    
    def receiveDone(self, message):
        if message.data  == 1: #all iterations are done. Check manager.py
            self.stop = True
        if message.data == 2: #timed out. Check manager.py
            self.task.restartProtocol(restart = 1)

    def load_nets(self):
        pass
    
    def saveModel(self):
        pass
    
    def get_action(self, s):
        # These are passed in manually in the v-rep simulation
        pass 

    def store(self, s):
        print(self.time)
        if self.mode == 'Spectral':
            s = s[:, 1:]
        s = np.hstack((s, np.array([self.time]).reshape(1,-1)))
        if self.time == 1:
            self.data = s 
        else:
            self.data = np.vstack((self.data, s))
        self.time += 1

        

    def reset(self):
        return 
        
    def train(self):
        pass

        
            
