#! /usr/bin/env python

from task import Task, unitVector, dot, vector
from task import distance as dist
from graphMap import Graph
import numpy as np 
import math
import rospy
import torch 
import torch.nn as nn
import vrep
import time
import copy
import sys
from Algs.doubleQ import DoubleQ
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from hierarchyTask import HierarchyTask 
from box_slope_task import BoxSlopeTask
from probabilisticModel import Model
from Algs.doubleQ import DoubleQ
from collections import OrderedDict
from sklearn.neighbors import KNeighborsClassifier as KNN
from sklearn.discriminant_analysis import QuadraticDiscriminantAnalysis as QDA
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, RationalQuadratic
from scipy.stats import chi, multivariate_normal, mode
from data_analysis import Analysis

load_paths = {
}

class Planner(object):
    def __init__(self, network_parameters, name):
        og_params = copy.deepcopy(network_parameters)

        self.fail = rospy.Publisher("/restart", Int8, queue_size = 1)

        self.agents = network_parameters['agents']
        self.net_params = network_parameters['valPars']
        self.train_params = network_parameters['valTrain']
        self.pubs = OrderedDict()
        for key in self.agents.keys():
            bot = self.agents[key]
            self.pubs[key] = rospy.Publisher(bot['pub'], Vector3, queue_size = 1)
        self.name = name

        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 
        self.changePoint = rospy.Publisher('/phaseChange', Int8, queue_size=1)

        self.action_map = {0: 'APPROACH', 1: 'ANGLE_TOWARDS', 2: 'PUSH_IN', 3: 'ALIGN_Y',
                           4: 'PUSH_LEFT', 5: 'PUSH_RIGHT', 6: 'MOVE_BACK', 7:'ANGLE_TOWARDS_GOAL'}

        self.goalSphere = None # (point, radius)
        self.prevPrim = None 
        self.reset_sequence = True

        self.max_num_actions = 5
        self.max_num_rollouts = 1
        self.MPC = False    
        self.GP = False
        self.mode = "Analysis" #'Plan' #

        self.box_policy = DoubleQ(network_parameters, self.name, HierarchyTask(), load_path ='/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Hybrid/Combination_Gaussian/hierarchical_q_policy2.txt')
        # self.model = Network(self.vPars, self.vTrain)
        self.controller = HierarchyTask()

        self.analysis = Analysis(self.mode)
        self.success, self.success_prime, self.fail, self.fail_prime, self.id_to_primitive, self.primitive_to_id = self.analysis.get_data()
        
        success_start, fail_start = self.analysis.get_start_data()
        if self.mode == 'Analysis':
            self.analysis.analyze_requested_data()
            sys.exit(0)

        self.train_model()
        while(True):
            x = 1+1
    
    
    def initAgent(self, agent):
        pass 
    
    def train_model(self):
        states = np.vstack((self.success, self.fail))
        results = np.vstack((self.success_prime, self.fail_prime))
        targets = results - states 

        losses = []
        for i in range(500):
            batch = np.random.choice(states.shape[0], size=512)
            s = states[batch, :]
            s_delta_tar = targets[batch, :]
            delta = self.model(torch.FloatTensor(s))
            loss = self.model.get_loss(delta, s_delta_tar)
            self.model.optimizer.zero_grad()
            loss.backward()
            self.model.optimizer.step()
            losses.append(loss)
        plt.plot(range(len(losses)), losses)
        plt.show()
        

    def concatenate_identifier(self, s):
        return np.hstack((s, np.repeat(1, s.shape[0]).reshape(-1,1)))

    def sendActionForPlan(self, states, phase):
        s = states['feature']
        if dist(s[:3], s[5:8]) < .3:
            msg = Int8()
            msg.data = 1
            self.changePoint.publish(msg)
        action_index = self.box_policy.get_action(self.concatenate_identifier(s))

        self.controller.goal = s.ravel()[:2]
        action = self.controller.getPrimitive(self.controller.feature_2_task_state(s.ravel()), self.action_map[action_index])
        
        msg.x, msg.y = (action[0], action[1])
        self.pubs[self.name].publish(msg)
        return 
    
    def stop_moving(self):
        dummy_task = self.primitive_tasks['slope_push']
        dummy_task.pubs = self.pubs
        dummy_task.name = self.name
        dummy_task.stop_moving()

    
    def split_state(self, s):
        states = {}
        states['slope'] = self.primitive_tasks['slope_push'].feature_joint_2_joint(np.hstack((s[:9], s[10:19], s[20:22])))
        states['feature'] = np.hstack((s[:5], s[6:10]))
        states['feature_next'] = np.hstack((s[:5], s[22:26]))
        states['flat'] = self.primitive_tasks['cross'].feature_2_task_state(states['feature'])
        return states

    def receiveState(self, msg):    
        if self.mode == 'Plan':
            floats = vrep.simxUnpackFloats(msg.data)
            floats = np.array(floats).ravel()
            phase = floats[-1]

            states = self.split_state(floats[:-1])
            a = self.sendActionForPlan(states, phase)  
            return 
    
    def restartProtocol(self, restart):
        if restart == 1:      
            msg = Int8()
            msg.data = 1
            self.curr_rollout = []
            self.fail.publish(msg)

    ######### POST TRAINING #########
    def postTraining(self):
        return 