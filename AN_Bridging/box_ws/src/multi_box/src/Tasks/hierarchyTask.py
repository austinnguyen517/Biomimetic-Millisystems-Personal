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
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt  
import pickle
import sys  

class HierarchyTask(Task):
    def __init__(self):
        super(HierarchyTask, self).__init__()
        self.prev = {"S": None, "A": None}
        self.primitive = 'REORIENT' #  NOTE: Make sure to set this tosomething other than cross for hierarchical MBRL
        if self.primitive == 'PUSH_IN_HOLE' or self.primitive == 'PUSH_TOWARDS' or self.primitive == 'SLOPE_PUSH':
            self.actionMap = {0: "APPROACH", 1: "ANGLE_TOWARDS", 2:"MOVE_BACK", 3: "ALIGN_Y", 4: "PUSH_IN"} 
        elif self.primitive == 'CROSS':
            self.actionMap = {0: "ANGLE_TOWARDS", 1: "MOVE_BACK", 2: "ALIGN_Y", 3: "CROSS"} #push in represents cross 
        elif self.primitive == 'REORIENT':
            self.actionMap = {0: 'MOVE_BACK', 1: 'PUSH_LEFT', 2: 'PUSH_RIGHT', 3: 'ALIGN_Y', 4: 'ANGLE_TOWARDS'}
            
        self.fail = rospy.Publisher("/restart", Int8, queue_size = 1)

        self.travel_gain = 2.5  
        self.align_gain = 8
        self.rotate_gain = 3
        self.x_contact = 0
        self.contact = {'left': .6, 'right': -.6}
        self.s_n = 7
        self.currReward = 0
        self.rewards = []

        self.curr_rollout = []
        self.data = []
        self.curr_size = 0
        self.restart_timer = True

        self.counter = 0
        self.period = 20 # TEST 20  
        self.mode = ""#'GET_STATE_DATA' 
        # TEST:
        self.commands = [3, 3, 3,3,3,3,3,3,3, 0, 0, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
    
    def resetPrimitive(self, primitive):
        self.primitive = primitive 
        if self.primitive == 'PUSH_IN_HOLE' or self.primitive == 'PUSH_TOWARDS':
            self.actionMap = {0: "APPROACH", 1: "ANGLE_TOWARDS", 2:"MOVE_BACK", 3: "ALIGN_Y", 4: "PUSH_IN"} 
        elif self.primitive == 'CROSS':
            self.actionMap = {0: "ANGLE_TOWARDS", 1: "MOVE_BACK", 2: "ALIGN_Y", 3: "CROSS"} #push in represents cross 
        elif self.primitive == 'REORIENT':
            self.actionMap = {0: 'MOVE_BACK', 1: 'PUSH_LEFT', 2: 'PUSH_RIGHT', 3: 'ALIGN_Y', 4: 'ANGLE_TOWARDS'}

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
            s_input = self.getNetInput(mock_s) if mock_s else self.getNetInput(s)
            ret = self.agent.get_action(s_input)
            # TEST:
            # ret = self.commands.pop(0)
            # print(self.actionMap[ret])
        else:
            ret = self.prev['A']
        action = self.getPrimitive(s, self.actionMap[ret])
        msg.x, msg.y = (action[0], action[1])
        self.pubs[self.name].publish(msg)
        return ret
    
    def stop_moving(self):
        msg = Vector3()
        msg.x, msg.y = (0, 0)
        self.pubs[self.name].publish(msg)
        print(' attemppting stop momving')
    
    def getPrimitive(self, s, a):
        # given state, action description and goal, return action representing left/right frequencies
        s = np.array(s).ravel()
        goal_angles, align_y_angles, cross_angles, left_angle, right_angle = self.getAngles(s)
        theta, phi = goal_angles
        alpha, beta, from_align = align_y_angles
        goal1, goal2 = cross_angles

        s = s.ravel()
        #diff_r = (np.linalg.norm(self.goal[:2] - s[:2]) - self.radius)
        if a == "APPROACH": 
            #action = [self.travel_gain * phi + (s[1] - self.y_box), self.travel_gain * theta + (self.y_box - s[1])]
            action = [self.travel_gain * phi, self.travel_gain * theta]
        if a == "ANGLE_TOWARDS":
            action = [self.rotate_gain * np.cos(theta), -self.rotate_gain * np.cos(theta)]
        if a == "MOVE_BACK":
            #action = [-self.travel_gain*theta, -self.travel_gain * phi]    
            action = [-self.travel_gain / 2, -self.travel_gain / 2]
        if a == "ALIGN_Y":
            action = [self.align_gain * beta * from_align, self.align_gain * alpha  * from_align]
        if a == "PUSH_IN" or a == "CROSS":
            action = [self.travel_gain * goal2, self.travel_gain * goal1]
        if a == 'PUSH_LEFT':
            action = [self.travel_gain * left_angle[1], self.travel_gain * left_angle[0]]
        if a == 'PUSH_RIGHT':
            action = [self.travel_gain * right_angle[1], self.travel_gain * right_angle[0]]    
        if a == 'ANGLE_TOWARDS_GOAL':
            action = [self.rotate_gain * np.cos(goal1), -self.rotate_gain * np.cos(goal1)] 
        return action
    
    def getAngles(self, s):
        s = s.ravel()
        goal = self.goal # This is relative position of box w.r.t. the robot

        if self.primitive == 'PUSH_IN_HOLE' or self.primitive == 'REORIENT' or self.primitive == 'PUSH_TOWARDS' or self.primitive == 'SLOPE_PUSH':
            relative_y = goal[0]
            relative_x = -goal[1]
        if self.primitive == 'CROSS':
            relative_y = s[4]
            relative_x = -s[5]
        buff = (-np.pi if relative_y < 0 else np.pi) if relative_x < 0 else 0 # since we want to map -pi to pi
        theta = np.arctan(relative_y/relative_x) + buff 
        phi = -np.pi - theta if theta < 0 else np.pi - theta  
        goal_angles = (theta, phi)

        # NOTE: Depending on the primitive, these all reference the box and some otherpoint past it as well 
        box_from_hole = s[:2] - s[4:6]
        hole = s[4:6]
        aligned = hole - dot(hole,unitVector(box_from_hole)) * unitVector(box_from_hole)
        relative_x = -aligned[1]
        relative_y = aligned[0]
        buff = (-np.pi if relative_y < 0 else np.pi) if relative_x < 0 else 0 # since we want to map -pi to pi
        alpha = np.arctan(relative_y/relative_x) + buff 
        beta = -np.pi - alpha if alpha < 0 else np.pi - alpha 
        align_y_angles = (alpha, beta, dist(aligned, np.zeros(2)))

        relative_y = s[4]
        relative_x = -s[5]
        buff = (-np.pi if relative_y < 0 else np.pi) if relative_x < 0 else 0 # since we want to map -pi to pi
        goal1 = np.arctan(relative_y/relative_x) + buff 
        goal2 = -np.pi - goal1 if goal1 < 0 else np.pi - goal1
        cross_angles = (goal1, goal2)

        pos = s[:2]
        psi = s[3]
        goal_relative_to_box = np.array([self.x_contact, self.contact['left']])
        rotation_matrix = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
        home = pos + rotation_matrix.dot(goal_relative_to_box)
        relative_y = home[0]
        relative_x = -home[1]
        buff = (-np.pi if relative_y < 0 else np.pi) if relative_x < 0 else 0 # since we want to map -pi to pi
        alpha = np.arctan(relative_y/relative_x) + buff 
        beta = -np.pi - alpha if alpha < 0 else np.pi - alpha 
        left_angle = (alpha, beta)

        goal_relative_to_box = np.array([self.x_contact, self.contact['right']])
        home = pos + rotation_matrix.dot(goal_relative_to_box)
        relative_y = home[0]
        relative_x = -home[1]
        buff = (-np.pi if relative_y < 0 else np.pi) if relative_x < 0 else 0 # since we want to map -pi to pi
        alpha = np.arctan(relative_y/relative_x) + buff 
        beta = -np.pi - alpha if alpha < 0 else np.pi - alpha 
        right_angle = (alpha, beta)

        return goal_angles, align_y_angles, cross_angles, left_angle, right_angle

    def checkConditions(self, full_state, a, complete = True):
        # given self.prev['A'] and state (unraveled already), check that we've sufficiently executed primitive
        if a == None:
            return True
        a = self.actionMap[a] if type(a) != str else a
        s = np.array(full_state).ravel()
        goal_angles, align_y_angles, cross_angles, left_angle, right_angle = self.getAngles(s)
        theta, phi = goal_angles
        alpha, beta, to_align = align_y_angles
        goal1, goal2 = cross_angles

        if a == "ANGLE_TOWARDS":
            return abs(theta - np.pi/2) < 5e-2 or self.counter == 0
        if a == "ANGLE_TOWARDS_GOAL":
            return abs(goal1 - np.pi/2) < 5e-2 or self.counter == 0
        if a == "ALIGN_Y":
            return to_align < .1 or self.counter == 0
        if a == "APPROACH":
            return dist(s[:2], np.zeros(2)) < .7 or self.counter == 0
        if a == 'PUSH_IN':
            if self.primitive == 'PUSH_IN_HOLE':
                return self.box_height < .35 or self.counter == 0
            else:
                return self.counter == 0

        return self.counter == 0

    def checkPhase(self, s):
        if self.primitive == 'PUSH_IN_HOLE' or self.primitive == 'CROSS':
            if self.rob_height < .35:
                return (-3, 1)
        if self.primitive == 'PUSH_IN_HOLE':
            if (self.box_height < .2):
                d = dist(s[:2], s[4:6])
                print('DISTANCE: ', d)
                if d < .2 and (self.hole_height > self.box_relative_height):
                    return (10 - d * 5, 1)
                else:
                    return (-3, 1)
        if self.primitive == 'CROSS':
            goal = s[4:6]
            d = dist(goal, np.zeros(2))
            if d < .2:
                print('distance: ', d)
                return (5, 1)
        if self.primitive == 'REORIENT':
            box_to_goal = s[4:6] - s[:2]
            goal_vector = unitVector(box_to_goal)
            goal_direction = math.atan(goal_vector[1]/goal_vector[0])
            curr_direction = s[3]
            d = dist(s[:2], s[4:6])
            if abs(self.box_y) > .8:# .2
                return (-3, 1)
            if abs(goal_direction - curr_direction) < .15 or d < .2:
                return (5 - abs(self.box_y), 1)
        if self.primitive == 'PUSH_TOWARDS':
            d = dist(s[:2], s[4:6])
            box_to_goal = s[4:6] - s[:2]
            goal_vector = unitVector(box_to_goal)
            goal_direction = math.atan(goal_vector[1]/goal_vector[0])
            curr_direction = s[3]
            if abs(self.box_ori) > .25 or abs(self.box_y) > .35:
                return (-2, 1)
            if d < .2: 
                return (5, 1)   
        if self.primitive == 'SLOPE_PUSH':
            d = dist(s[:2], s[4:6])
            box_to_goal = s[4:6] - s[:2]
            goal_vector = unitVector(box_to_goal)
            goal_direction = math.atan(goal_vector[1]/goal_vector[0])
            curr_direction = s[3]
            if abs(self.box_ori) > .6 or abs(self.box_y) > .5:
                return (-2, 1)
            if d < .2: 
                return (5, 1)   
        return (0,0)
    
    def getNetInput(self, s):
        s = s.reshape(1, -1)
        if self.primitive == 'PUSH_TOWARDS':
            return s[:, :-1]
        return s

    def rewardFunction(self, s, a):
        s = s.ravel()
        prevS = self.prev["S"].ravel()
        res = self.checkPhase(s)
        if res[0] != 0:
            return res
        # make the penalty larger...
        return (-.1, 0)

    def isValidAction(self, s, a):
        return not self.checkConditions(s.ravel(), a)
    
    def extract_feature(self, s):
        if self.primitive == 'SLOPE_PUSH':
            return np.hstack((s[:5], s[6:9], s[18]))
        return s 
    def feature_2_task_state(self, feature):
        return np.hstack((feature[:2], feature[3:7], feature[8:])) 
    
    def append_states(self):
        self.curr_size += len(self.curr_rollout)
        self.data.append(self.curr_rollout)

    def data_to_txt(self, path):
        with open(path, "wb") as fp:   #Pickling
            pickle.dump(self.data, fp)
        return 

    def receiveState(self, msg): 
        if self.restart_timer:
            self.start_time = time.time()
            self.restart_timer = False
        floats = vrep.simxUnpackFloats(msg.data)
        local_state = np.array(floats).ravel()
        local_state = self.extract_feature(local_state)
        feature = np.hstack((local_state[:9], np.array([0 if self.primitive == 'CROSS' else 1]))) # Consider making this lenght 8
        local_state = self.feature_2_task_state(local_state)
        self.goal = local_state[:2] # goal is position of box relative to robot 
        restart = 0
        r = None
        if self.primitive == 'SLOPE_PUSH':
            self.box_y = floats[20]
            self.box_ori = floats[18]
        else:
            self.box_height = local_state[self.s_n]
            self.rob_height = local_state[self.s_n + 1]
            self.hole_height = feature[7]
            self.box_relative_height = feature[2]
            self.box_y = local_state[self.s_n + 2]
            self.box_ori = local_state[self.s_n + 3]
        local_state = local_state[:self.s_n]

        changeAction = self.checkConditions(local_state, self.prev['A'], complete=False)
        s = (np.array(local_state)).reshape(1,-1)
        curr_s = self.getNetInput(s)
        if changeAction:
            self.counter = self.period
            a = (self.sendAction(s))
            if self.mode == 'GET_STATE_DATA' and self.isValidAction(s, a):
                self.curr_rollout.append(feature)
            if type(self.prev["S"]) == np.ndarray and type(self.prev["A"]) == int:
                r, restart = self.rewardFunction(s, self.prev["A"])
                r = r if self.isValidAction(self.prev['S'], self.prev['A']) or restart else -.1 # make the penalty larger
                if restart: 
                    if r > 0:
                        print(' #### Success!')
                    else:
                        print(' #### Failed')
                    
                prev_s = self.getNetInput(self.prev['S'])
                self.agent.store(prev_s, self.prev["A"], np.array([r]).reshape(1, -1), curr_s, a, restart)
                if restart: 
                    print('Last transition recorded')
                self.currReward += r

            if self.trainMode:
                loss = self.agent.train()

            self.prev["S"] = s
            self.prev["A"] = int(a)
            l = len(self.agent.exp)
            if l <= 500:
                print('exp length', l)
        else:
            a = self.sendAction(s, changeAction)
            # SPECIAL CASE: since we execute one primitive for multiple time steps (with intermittent control updates), we need to store transitions/rewards when the agent fails out or succeeds
            if type(self.prev["S"]) == np.ndarray and type(self.prev["A"]) == int:
                r, restart = self.rewardFunction(s, self.prev["A"])
                if restart:
                    if r > 0: # we assume failure has rewards < 0
                        print(' #### Success!')
                    else:
                        print(' #### Dropped')
                    prev_s = self.getNetInput(self.prev['S'])
                    self.agent.store(prev_s, self.prev["A"], np.array([r]).reshape(1, -1), curr_s, a, restart)
                    print('Last transition recorded')
                    self.currReward += r
        if restart and r > 0 and self.mode == 'GET_STATE_DATA':
            self.curr_rollout.append(feature)
        self.restartProtocol(restart, succeeded=r > 0 if r else False)   
        return 
    
    def restartProtocol(self, restart, succeeded = False):
        if restart == 1:      
            print('Results:     Cumulative Reward: ', self.currReward, '    Steps: ', self.agent.totalSteps)
            print("")
            if self.mode == 'GET_STATE_DATA':       
                if len(self.curr_rollout) > 0:
                    time_execute = time.time() - self.start_time 
                    self.restart_timer = True 
                    print(' Succeded: ', succeeded, '    Time: ', time_execute)
                    self.curr_rollout.append(int(succeeded))
                    self.curr_rollout.append(time_execute)
                    self.append_states()
                    print(' LENGTH OF DATA: ', self.curr_size)
                    if self.curr_size > 1500:
                        self.agent.stop = True
            for k in self.prev.keys():
                self.prev[k] = None
            self.goal = 0
            if self.currReward != 0:
                self.rewards.append(self.currReward)
            self.currReward = 0
            self.curr_rollout = []
            self.agent.reset()
            msg = Int8()
            msg.data = 1
            self.fail.publish(msg)

    ######### POST TRAINING #########
    def postTraining(self):
        if self.mode == 'GET_STATE_DATA':
            self.data_to_txt(path =  '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/' +self.primitive + '_state_data_GP2_better.txt')
            sys.exit(0)
        else:
            self.agent.saveModel()
            self.plotRewards()

    def plotLoss(self, valueOnly = False, title1 = "Critic Loss over Iterations", title2 = "Actor Loss over Iterations"):
        x = range(len(self.agent.valueLoss))
        plt.plot(x, self.agent.valueLoss)
        plt.title(title1)
        plt.legend()
        window= np.ones(int(15))/float(15)
        line = np.convolve(self.agent.valueLoss, window, 'same')
        plt.plot(x, line, 'r')
        grid = True
        plt.show()  
        if not valueOnly:
            x = range(len(self.agent.actorLoss))
            window = np.ones(int(15))/float(15)
            line = np.convolve(self.agent.actorLoss, window, 'same')
            plt.plot(x, line, 'r')
            plt.plot(x, self.agent.actorLoss)
            plt.title(title2)
            plt.show()
    
    def plotRewards(self):
        print(len(self.rewards))
        x = range(len(self.rewards))
        plt.plot(x, self.rewards)
        plt.title("Rewards Over Episodes w/ Moving Average")
        plt.legend()
        window= np.ones(int(15))/float(15)
        lineRewards = np.convolve(self.rewards, window, 'same')
        plt.plot(x, lineRewards, 'r')
        grid = True
        plt.show()