#! /usr/bin/env python

class Task(object):
    def __init__(self, action):
        assert action == "argmax" or action == "p_policy"
        self.a = action
        self.init = False
        
    def initAgent(self, agent):
        if self.a == "argmax":
            self.valueNet = agent.valueNet
        if self.a == "p_policy":
            self.policyNet = agent.policyNet

        self.agent = agent 
        self.extractInfo()
        self.init = True  

    def extractInfo(self):
        pass

    def sendAction(self,s):
        pass
    
    def rewardFunction(self, s, a):
        pass

    def receiveState(self, msg):
        pass
    
    def restartProtocol(self, restart):
        pass

    def postTraining(self):
        pass