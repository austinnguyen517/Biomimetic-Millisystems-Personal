#! /usr/bin/env python

from task import Task, unitVector, dot, vector
from task import distance as dist
import numpy as np 
import math
import rospy
import torch 
import torch.nn as nn
import vrep
import time
from std_msgs.msg import String, Int8, Int16
from hierarchyTask import HierarchyTask
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt  
import pickle
import sys  

class Hierarchy_MBRL_Task(Task):
    def __init__(self):
        super(Hierarchy_MBRL_Task, self).__init__()
        self.prev = {"S": None, "A": None}
        self.fail = rospy.Publisher("/restart", Int8, queue_size = 1)
        rospy.Subscriber("/simulation", String, self.receive_simulation_description, queue_size = 1)
        rospy.Subscriber("/starting", Int16, self.receive_starting_cue, queue_size = 1)
        self.action_map = {0: 'APPROACH', 1: 'ANGLE_TOWARDS', 2: 'PUSH_IN', 3: 'ALIGN_Y',
                           4: 'PUSH_LEFT', 5: 'PUSH_RIGHT', 6: 'MOVE_BACK', 7:'ANGLE_TOWARDS_GOAL'}

        self.s_n = 9
        self.currReward = 0
        self.rewards = []
        self.testing_rewards = []

        self.curr_rollout = []
        self.data = []
        self.curr_size = 0
        self.curr_episode = (1, False)
        self.local_curr_episode_synchronous = 0
        self.done = True
        self.tracker_for_testing = 0

        self.counter = 0
        self.period = 50
        self.mode =  '' # 'GET_STATE_DATA' #
        self.controller = HierarchyTask()
        self.simulation_name = None
        self.prev_action_was_valid = True
   
    def extractInfo(self):
        self.vTrain = self.agent.vTrain
        self.pubs = self.agent.pubs
        self.trainMode = self.agent.trainMode
        self.name = self.agent.name
        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 
 
    def sendAction(self, s, changeAction=True, mock_s=None):
        msg = Vector3()
        self.counter -= 1
        if changeAction:
            ret = self.agent.get_action(s.reshape(1,-1))
            print(self.action_map[ret])
        else:
            ret = self.prev['A'][0]
        action = self.action_map[ret]
        self.controller.goal = s.ravel()[:2]
        action = self.controller.getPrimitive(self.controller.feature_2_task_state(s.ravel()), action)
        msg.x, msg.y = (action[0], action[1])
        self.pubs[self.name].publish(msg)

        adjusted_state_for_controls = self.controller.feature_2_task_state(s.ravel())
        self.prev_action_was_valid = True if self.isValidAction(adjusted_state_for_controls, self.action_map[ret]) else False 
        return ret, action # NOTE: we changed this so we coud get the raw differential drive output
    
    def stop_moving(self):
        msg = Vector3()
        msg.x, msg.y = (0, 0)
        self.pubs[self.name].publish(msg)
        #print(' attemppting stop momving')
    
    def changeAction(self, s, a, complete):
        self.controller.counter = 1
        return self.counter == 0 #or # self.controller.checkConditions(s, a, complete) 
        
    def isValidAction(self, s, a):
        self.controller.counter = 1
        return self.controller.isValidAction(s, a)

    def append_states(self):
        self.curr_size += len(self.curr_rollout)
        self.data.append(self.curr_rollout)

    def reward_function(self, s):
        s = s.ravel()
        succeeded = self.succeeded(s)
        done = self.decide_to_restart(s)
        if succeeded:
            if self.simulation_name == 'elevated_scene':
                return 10 - dist(s[:3], s[5:8]) * 5
            if self.simulation_name == 'flat_scene':
                return 10 - abs(self.box_ori_global) * 3
            if self.simulation_name == 'slope_scene':
                return 10 - abs(self.box_ori_global) * 3
        if done and not succeeded:
            return -3
        else:
            if self.prev_action_was_valid:
                return -.25
            else:
                return -.4
    
    def receive_starting_cue(self, msg):
        self.curr_episode = (msg.data, False)
    
    def receive_simulation_description(self, msg):
        self.simulation_name = msg.data

    def receiveState(self, msg): 
        if self.curr_episode[0] >= self.local_curr_episode_synchronous + 1 and self.curr_episode[1] == False:
            print('ENVIRONMENT: ', self.simulation_name)
            self.done = False
            self.start_time = time.time()
            # Train for 20 episode, Test for 20 episode
            if self.agent.testing_to_record_progress and self.tracker_for_testing % 20 == 0:
                self.agent.testing_to_record_progress = False
                self.tracker_for_testing = 0
            elif not self.agent.testing_to_record_progress and self.tracker_for_testing % 20 == 0:
                self.agent.testing_to_record_progress = True
                self.tracker_for_testing = 0

            self.local_curr_episode_synchronous += 1
            self.tracker_for_testing += 1

            if self.agent.testing_to_record_progress:
                print('  ##### TESTING ##### ')
            else:
                print('  ##### TRAINING ##### ')

        floats = vrep.simxUnpackFloats(msg.data)
        self.bot_z_global = floats[self.s_n + 1]
        self.box_z_global = floats[self.s_n]
        self.box_y_global = floats[-2]
        self.box_ori_global = floats[-1]
        local_state = np.array(floats[:self.s_n]).ravel()
        
        adjusted_state_for_controls = self.controller.feature_2_task_state(local_state)
        changeAction = self.changeAction(adjusted_state_for_controls, self.action_map[self.prev['A'][0]], complete=False) if type(self.prev['A']) == tuple else True
        s = (np.array(local_state)).reshape(1,-1)

        succeeded = self.succeeded(s.ravel())
        restarting = self.decide_to_restart(s.ravel())
        self.done = restarting or succeeded

        reward = self.reward_function(s)
        if not self.curr_episode[1]:
            if not self.done:
                if changeAction:
                    # print(int(self.done), reward)
                    self.counter = self.period
                    action_index, action_control = (self.sendAction(s, changeAction))
                    if self.isValidAction(adjusted_state_for_controls, self.action_map[action_index]):
                        self.curr_rollout.append(s.ravel())
                    if type(self.prev["S"]) == np.ndarray and not self.agent.testing_to_record_progress:
                        print(self.action_map[self.prev['A'][0]], reward)
                        self.agent.store(self.prev['S'], np.array(self.prev["A"][0]), reward, s, 0, self.done, self.prev['A'][0])
                    if self.trainMode:
                        loss = self.agent.train(self.curr_episode[0])

                    self.prev["S"] = s
                    self.prev["A"] = (int(action_index), action_control)
                    if not self.curr_episode[1]:
                        self.currReward += reward
                else:
                    action_index, a = self.sendAction(s, changeAction)
            else:        
                if type(self.prev["S"]) == np.ndarray:
                    prev_s = self.prev['S']
                    if not self.agent.testing_to_record_progress:
                        self.agent.store(prev_s, np.array(self.prev["A"][0]), reward, s, 0, self.done, self.prev['A'][0])
                        print('Last transition recorded')

                self.currReward += reward   
                if succeeded:
                    assert reward > 0
                    print(' ##### SUCCESS ')
                    print(' ##### Success reward: ', reward)
        
        self.restartProtocol(self.done , succeeded=succeeded)   
        return 
    
    def succeeded(self, s):
        if self.simulation_name == 'elevated_scene':
            return dist(s[:3], s[5:8]) < .5 and self.box_z_global < .2 and self.bot_z_global > .3
        if self.simulation_name == 'flat_scene':
            return dist(s[:3], s[5:8]) < .4
        if self.simulation_name == 'slope_scene':
            return dist(s[:3], s[5:8]) < .4

    def decide_to_restart(self, s):
        # if far away from box, far away from goal, box dropped, or bot dropped
        if self.simulation_name == 'elevated_scene':
            return dist(s[:3], np.zeros(3)) > 3.5 or dist(s[5:8], np.zeros(3)) > 3.5  or self.box_z_global < .2 or self.bot_z_global < .3 or self.currReward <= -20
        if self.simulation_name == 'flat_scene':
            return dist(s[:3], np.zeros(3)) > 3.5 or dist(s[5:8], np.zeros(3)) > 4 or abs(self.box_y_global) > 1 or self.currReward <= -20
        if self.simulation_name == 'slope_scene':
            return abs(self.box_ori_global) > .4 or dist(s[:3], np.zeros(3)) > 2 or self.currReward <= -20
        
    
    def restartProtocol(self, restart, succeeded = False):
        if restart == 1 and self.curr_episode[0] > (len(self.testing_rewards) + len(self.rewards)): 
            self.curr_episode = (self.curr_episode[0], self.done)
            print(' ######## Episode Reward: ', self.currReward)
            print('')
            if self.trainMode:
                if self.agent.testing_to_record_progress:
                    self.testing_rewards.append(self.currReward)
                else:
                    self.rewards.append(self.currReward)
                    if succeeded:
                        self.agent.successful_append(self.curr_rollout)
            if not self.trainMode:
                self.testing_rewards.append(self.currReward)
            if self.mode == 'GET_STATE_DATA':       
                if len(self.curr_rollout) > 0:
                    time_execute = time.time() - self.start_time 
                    print(' Succeded: ', succeeded, '    Time: ', time_execute)
                    self.curr_rollout.append(int(succeeded))
                    self.curr_rollout.append(time_execute)
                    self.append_states()
                    print(' LENGTH OF DATA: ', self.curr_size)
                    if self.curr_size > 5000:
                        self.agent.stop = True
            for k in self.prev.keys():
                self.prev[k] = None
            self.currReward = 0
            self.curr_rollout = []
            self.agent.reset()
            msg = Int8()
            msg.data = 1
            self.fail.publish(msg)
            if self.curr_episode[0] % 50 == 0 and self.curr_episode[0] < 1200 and self.curr_episode[0] != 0:   
                self.agent.train_model()


    def data_to_txt(self, data, path):
        with open(path, "wb") as fp:   #Pickling
            pickle.dump(data, fp)
        return 

    ######### POST TRAINING #########
    def postTraining(self):
        if self.mode == 'GET_STATE_DATA':
            self.data_to_txt(data = self.data, path =  '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/' + self.agent.method + '_state_data.txt')
            self.data_to_txt(data = self.testing_rewards, path = '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/' + self.agent.method + '_post_training_testing_rewards.txt')
            sys.exit(0)
        else:
            self.agent.saveModel()
            self.data_to_txt(data = self.rewards, path = '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/rewards.txt')
            self.data_to_txt(data = self.testing_rewards, path = '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/testing_rewards.txt')
            self.data_to_txt(data = self.agent.loss, path =  '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/model_loss.txt')
            #self.plotRewards()
            #self.plotLoss()

    
    def plotRewards(self):
        x = range(len(self.rewards))
        plt.plot(x, self.rewards)
        plt.title("Rewards Over Episodes w/ Moving Average")
        plt.legend()
        window= np.ones(int(15))/float(15)
        lineRewards = np.convolve(self.rewards, window, 'same')
        plt.plot(x, lineRewards, 'r')
        grid = True
        # Note: we don't care about the training rewards
        # plt.show()
    
    def plotLoss(self):
        loss = self.agent.loss
        plt.plot(range(len(loss)), loss, label='Training')
        if len(self.agent.validation_loss) > 0:
            validation_loss = self.agent.validation_loss
            plt.plot(range(len(validation_loss)), validation_loss, 'r', label='Validation')
        plt.legend()
        plt.show() 