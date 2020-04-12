#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import math 
from network import Network
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt

class Agent(object):
    def __init__(self, params, name, task):
        self.name = name
        self.task = task
        self.vPars = params['valPars']
        self.vTrain = params['valTrain']
        self.agents = params['agents']
        self.valueNet = Network(self.vPars, self.vTrain)
        rospy.Subscriber("/finished", Int8, self.receiveDone, queue_size = 1)
        self.pubs = {}
        for key in self.agents.keys():
            bot = self.agents[key]
            self.pubs[key] = rospy.Publisher(bot['pub'], Vector3, queue_size = 1)
        rospy.Subscriber(self.agents[self.name]['sub'], String, task.receiveState, queue_size = 1) 

        self.batch_size = self.vTrain['batch']
        self.discount = self.vTrain['gamma']
        self.state_n = self.vPars['in_n']
        self.prob = self.vPars['prob']
        self.u_n = self.vPars['u_n']
        self.trainMode = self.vPars['trainMode']

        self.dataSize = 0 #number of data tuples we have accumulated so far
        self.stop = False
    
    def receiveDone(self, message):
        if message.data  == 1: #all iterations are done. Check manager.py
            self.stop = True
        if message.data == 2: #timed out. Check manager.py
            task.restartProtocol(restart = 1)
    
    def train(self):
        pass 
    
    def store(self, s, a, r, sprime, aprime = None):
        pass 
    
    def saveModel(self):
        pass
    
