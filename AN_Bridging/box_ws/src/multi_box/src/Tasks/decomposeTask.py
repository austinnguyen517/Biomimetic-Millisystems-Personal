#! /usr/bin/env python

from task import Task, unitVector, dot, vector
from task import distance as dist
import numpy as np 
import rospy
import vrep
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt

class DecomposeTask(Task):
    def __init__(self):
        super(DecomposeTask, self).__init__()

    def extractInfo(self):
        self.pubs = self.agent.pubs
        self.name = self.agent.name
        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 

    def receiveState(self, msg):
        floats = vrep.simxUnpackFloats(msg.data)
        restart = floats[-1]
        floats = floats[:-1]

        s = (np.array(floats)).reshape(1,-1)
        self.agent.store(s)

        self.restartProtocol(restart)
        return 
    
    def restartProtocol(self, restart):
        if restart == 1:
            self.agent.reset()

    ######### POST TRAINING #########
    def postTraining(self):
        self.plotTrajectory()
    
    def plotTrajectory(self):
        data = self.agent.data[:, :2]
        # Assumption: the first two columns of data represent x and y coordinates
        labels = self.agent.labels
        plt.scatter(data[:,0], data[:,1], c=labels, s=100)
        plt.show()
    