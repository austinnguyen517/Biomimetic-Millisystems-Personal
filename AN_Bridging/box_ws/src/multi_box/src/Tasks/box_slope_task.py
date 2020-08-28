#! /usr/bin/env python

from task import Task, unitVector, dot, vector
from task import distance as dist
import numpy as np 
import math
import rospy
import torch 
import time
import torch.nn as nn
import vrep
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
import pickle
import sys
    

class BoxSlopeTask(Task):
    def __init__(self):
        super(BoxSlopeTask, self).__init__()
        self.actionMap = {0: "STRAIGHT_BOX", 1: "HOME_CONTACT", 2: "CHANGE_ANGLE", 3: "BACK"} 
        self.forward_gain = 2
        self.home_gain = 2
        self.adjust_gain = 3
        self.back_gain = -1
        self.num_agents = 2
        self.prev = {"S": [None for i in range(self.num_agents)], "A": [None for i in range(self.num_agents)]}
        self.prevAngles = {"robot" + str(i): None for i in range(self.num_agents)}
        self.fail = rospy.Publisher("/restart", Int8, queue_size = 1)

        self.currReward = 0
        self.rewards = []
        self.explicit_control = False

        self.period = 20
        self.angle_threshold = .05
        self.counter = [0 for i in range(self.num_agents)]
        
        #self.goal_height = 1.5
        self.x_contact = -.2
        #self.contact = [.8 - i*(1.6)/(self.num_agents - 1) for i in range(self.num_agents)]

        self.mode = 'GET_STATE_DATA'
        self.curr_rollout1 = []
        self.curr_rollout2 = []
        self.data = []
        self.curr_size = 0
        self.restart_timer = True

        self.goal_height = 2            
        self.contact = [.4 - i*(.8)/(self.num_agents - 1) for i in range(self.num_agents)]

    def extractInfo(self):
        self.vTrain = self.agent.vTrain
        self.pubs = self.agent.pubs
        self.trainMode = self.agent.trainMode
        self.name = self.agent.name
        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 
 
    def sendAction(self, s):
        msg = Vector3()
        self.changeAction = self.checkConditions(s, self.prev['A'])
        angles = self.getAngles(s)
        states = self.splitState(s)
        ret = []
        for i, name in enumerate(self.pubs.keys()):
            loc = "robot" + str(i)
            angle = angles[loc]
            state = states[loc]
            if self.changeAction[i]:
                self.counter[i] = self.period
                action = self.getNextAction(state, angle, i)
                print(i, self.actionMap[action])
            else: 
                action = self.prev['A'][i]
            ret.append(action)
            action = self.getPrimitive(angle, self.actionMap[action], i)
            msg.x, msg.y = (action[0], action[1])
            self.pubs[name].publish(msg)
            self.counter[i] -= 1
        return ret
    
    def stop_moving(self):
        msg = Vector3()
        msg.x, msg.y = (0, 0)
        for i, name in enumerate(self.pubs.keys()):
            self.pubs[name].publish(msg)
        print(' stopping to plan')
    
    def getNextAction(self, state, angle, i):
        '''Order of priority: Box orientation (theta), Contact Point, Agent orientation'''
        theta = angle[3]
        tilt = angle[4]
        ori = angle[2]
        if self.explicit_control: #outdated
            to_contact = dist(state[:2], np.array([self.x_contact, self.contact[i]]))
            if theta > self.angle_threshold: #you're ahead or box is slipping away
                action = 3
            elif to_contact > .5: # far from your contact point 
                action = 1
            elif abs(ori) > .5: # orientation incorrect
                action = 2
            elif tilt:
                action = 3
            else: 
                action = 0  # for now just push
            return action
        else:
            # In the state information: relative position to contact point, theta, orientation angle. Total: 4
            return self.agent.get_action(self.getNetInput(state, angle, i))
    
    def getNetInput(self, state, angle, i):
        theta = angle[3]
        phi = angle[2]
        z = state[3] / (self.goal_height - .1)
        goal_relative_to_box = np.array([self.x_contact, self.contact[i]])
        rotation_matrix = np.array([[np.cos(-phi), -np.sin(-phi)], [np.sin(-phi), np.cos(-phi)]])
        relative_position = state[:2] + rotation_matrix.dot(goal_relative_to_box)
        s = np.hstack((relative_position, theta, phi, z, np.sign(self.contact[i]))) 
        return s
    
    
    def getPrimitive(self, angle, a, i): 
        # Given a local state s, action description a and distance of contact point from center of box dist
        alpha, beta, phi, box_ori, tilt = angle 
        damp = abs(self.contact[i] / max(self.contact))
        if a == "STRAIGHT_BOX":
            action = np.array([self.forward_gain - damp*box_ori + np.sin(phi) * 5, self.forward_gain - damp*box_ori - np.sin(phi) * 5])
        if a == "HOME_CONTACT":
            action = np.array([self.home_gain*beta, self.home_gain*alpha])
        if a == "CHANGE_ANGLE":
            action = [self.adjust_gain * np.sin(phi), -self.adjust_gain * np.sin(phi)]
        if a == "BACK":
            action = [self.back_gain, self.back_gain]
        return action
    
    def splitState(self, s):
        s = np.array(s).ravel()
        states = {}
        for i in range(self.num_agents):
            start = i * 4
            end = (i + 1) * 4
            key = "robot" + str(i)
            states[key] = s[start: end]
        states["box"] = s[self.num_agents * 4:]
        return states
    
    def getAngles(self, s):
        states = self.splitState(s)
        box = states['box']
        box_ori = box[0]

        angles = {}

        for i in range(self.num_agents):


            curr = states["robot" + str(i)]
            pos = curr[:2]
            phi = -curr[2]

            # Calculating angle for STRAIGHT_BOX and CHANGE_ANGLE
            direction = -1 if self.contact[i] > 0 else 1
            
            # Special case: if you're smack in the middle, you should go back if either side is uneven
            direction = np.sign(box_ori) if self.contact[i] == 0 else direction
            
            goal_relative_to_box = np.array([self.x_contact, self.contact[i]])
            rotation_matrix = np.array([[np.cos(-phi), -np.sin(-phi)], [np.sin(-phi), np.cos(-phi)]])
            home = pos + rotation_matrix.dot(goal_relative_to_box)


            # Calculating angles for HOME_CONTACT
            relative_y = home[0]
            relative_x = -home[1]
            buff = (-np.pi if relative_y < 0 else np.pi) if relative_x < 0 else 0 # since we want to map -pi to pi
            alpha = np.arctan(relative_y/relative_x) + buff 
            beta = -np.pi - alpha if alpha < 0 else np.pi - alpha 
            contact_angles = (alpha, beta)

            # Boolean to determine need for translation adjustment ie. if box is too far in, tilt it towards you
            '''ratio = abs(goal[1]/goal[0]) # ratio  horizontal and vertical distance from contact point 
            tilt = True if ratio > .8 and np.sign(goal[1] * self.contact[i]) < 0 and abs(goal[0]) < .4 else False '''
            tilt = None

            angles['robot' + str(i)] = (alpha, beta, phi, direction*box_ori, tilt)
        
        return angles

    def checkConditions(self, full_state, a):
        # given self.prev['A'] and state (unraveled already), check that we've sufficiently executed primitive
        fill = []
        states = self.splitState(full_state)
        prev_states = self.splitState(self.prev['S'])
        angles = self.getAngles(full_state)
        box = states["box"]
        for i in range(self.num_agents):
            if a[0] == None:
                fill.append(True)
                continue
            k = "robot" + str(i)
            angle = angles[k]
            theta = angle[3]
            act = self.actionMap[a[i]]
            curr = states[k]
            if act == "STRAIGHT_BOX":
                fill.append(self.timeOut(i))
            if act == "HOME_CONTACT":
                if self.explicit_control:
                    fill.append(self.timeOut(i) or self.fallingBehind(theta) or dist(curr[:2], np.array([self.x_contact, self.contact[i]])) < .35)
                else:
                    fill.append(self.timeOut(i) or dist(curr[:2], np.array([self.x_contact, self.contact[i]])) < .35)
            if act == "CHANGE_ANGLE":
                if self.explicit_control:
                    fill.append(self.timeOut(i) or self.fallingBehind(theta) or abs(box[2] - curr[2]) < .05)
                else:
                    fill.append(self.timeOut(i) or abs(box[0] - curr[2]) < .05)    
            if act == "BACK":
                fill.append(self.timeOut(i))
        return fill

    def isValidAction(self, s, a, angle, i):
        act = self.actionMap[a]
        ori = angle[2]
        if act == "STRAIGHT_BOX":
            return True 
        if act == "HOME_CONTACT" :
            return not (dist(s[:2], np.array([self.x_contact, self.contact[i]])) < .35)
        if act == "CHANGE_ANGLE":
            return not abs(ori) < .05
        if act == "BACK":
            return True
        return
    
    def timeOut(self, i):
        return self.counter[i] == 0

    def fallingBehind(self, theta):
        return theta < -self.angle_threshold

    def getAux(self, pos, prevPos, blockPos, prevBlock, ori, prevOri, phase, blockOri):
        return 

    def checkPhase(self, s):
        s = s.ravel()
        theta = s[2]
        phi = s[3]
        z = s[4]
        to_contact = dist(s[:2], np.zeros(2))
        if theta > .45 or to_contact > 1.2:
            print('#### Failed out')
            return (-3, 1)
        if self.box_height >= self.goal_height and self.dist_box_from_goal < 1:
            print('#### SUCCESS')
            return (10, 1)
        return (0, 0)

    def rewardFunction(self, s, prevS, i):
        s = s.ravel()
        theta = s[2]
        phi = s[3]
        z = s[4]

        res = self.checkPhase(s)
        if res[1] == 1:
            return res

        prev_theta = prevS[2]
        prev_phi = prevS[3]
        prev_z = prevS[4]

        delta_theta = np.round(abs(prev_theta) - abs(theta), 3)
        delta_ori = np.round(abs(prev_phi) - abs(phi), 3)
        delta_z = np.round(z - prev_z, 3)

        r = delta_z * 3 + delta_ori + delta_theta

        return (r, 0)
    

    def feature_joint_2_feature(self, feature):
        # Returns multiple feature vectors
        first_feature = np.hstack((feature[:5], feature[6:9], feature[18: 19], np.array([1])))
        second_feature = np.hstack((feature[9:14], feature[15:19], np.array([1])))
        return [first_feature, second_feature]

    def feature_joint_2_joint(self, feature_joint):
        # Returns one oint vector without extraneous features
        return np.hstack((feature_joint[:2], feature_joint[4:6], feature_joint[9:11], feature_joint[13:15], feature_joint[18:20]))

    def append_states(self):
        if len(self.curr_rollout1) > 5 and len(self.curr_rollout2) > 5:
            self.curr_size += len(self.curr_rollout1) + len(self.curr_rollout2)
            self.data.append(self.curr_rollout1)
            self.data.append(self.curr_rollout2)
        else:
            print(' ### Not recorded. Too short')

    def data_to_txt(self, path):
        with open(path, "wb") as fp:   #Pickling
            pickle.dump(self.data, fp)
        return 

    def receiveState(self, msg):     
        if self.restart_timer:
            self.start_time = time.time()
            self.restart_timer = False
        floats = vrep.simxUnpackFloats(msg.data)
        restart = 0
        r = None 
        self.dist_box_from_goal = dist(np.array(floats[0:2]), np.array(floats[6:8]))            
        features = self.feature_joint_2_feature(floats)
        floats = self.feature_joint_2_joint(np.array(floats).ravel())
        s = (np.array(floats)).reshape(1,-1)
        angles = self.getAngles(s)
        states = self.splitState(s)
        self.box_height = states['box'][-1] 
        a = (self.sendAction(s))
        if self.mode == 'GET_STATE_DATA':
            if self.changeAction[0]:
                self.curr_rollout1.append(features[0])
            if self.changeAction[1]:
                self.curr_rollout2.append(features[1])
        rest = 0
        for i in range(self.num_agents):
            loc = 'robot' + str(i)
            angle = angles[loc]
            curr_state = self.getNetInput(states[loc], angle, i)
            if type(self.prev['S'][i]) == np.ndarray and type(self.prev['A'][i]) == int:
                prevAngle = self.prevAngles[loc]
                prev_state = self.getNetInput(self.prev['S'][i], prevAngle, i)
                r, restart = self.rewardFunction(curr_state, prev_state, i)
                rest = restart if restart == 1 else rest
                if self.changeAction[i] or rest:
                    r = r if self.isValidAction(self.prev['S'][i], self.prev['A'][i], angle, i) or rest else -1
                    self.agent.store(prev_state, self.prev["A"][i], np.array([r]).reshape(1, -1), curr_state, a[i], restart)
                    self.currReward += r
            if self.changeAction[i]:    
                self.prev["S"][i] = states[loc]
                self.prev["A"][i] = int(a[i])
                self.prevAngles[loc] = angles[loc]
        if any(self.changeAction) and self.trainMode:
            loss = self.agent.train()
        if restart and r > 0 and self.mode == 'GET_STATE_DATA':
            self.curr_rollout1.append(features[0])
            self.curr_rollout2.append(features[1])
        self.restartProtocol(rest, succeeded=r > 0 if r else False)
        return 
    
    def restartProtocol(self, restart, succeeded=False):
        if restart == 1:   
            if self.mode == 'GET_STATE_DATA':
                time_execute = time.time() - self.start_time 
                self.restart_timer = True

                print('Record as success: ', succeeded)
                self.curr_rollout1.append(int(succeeded))
                self.curr_rollout2.append(int(succeeded))
                print('Time to execute: ', time_execute)
                self.curr_rollout1.append(time_execute)
                self.curr_rollout2.append(time_execute)
                self.append_states()
                print(' LENGTH OF DATA: ', self.curr_size)
                if self.curr_size > 1000:
                    self.agent.stop = True
            msg = Int8()
            msg.data = 1
            self.fail.publish(msg) 
            print('Results:     Cumulative Reward: ', self.currReward, '    Steps: ', self.agent.totalSteps)
            print("")
            for k in self.prev.keys():
                self.prev[k] = [None for i in range(self.num_agents)]
            self.goal = 0
            if self.currReward != 0:
                self.rewards.append(self.currReward)
            self.curr_rollout1 = []
            self.curr_rollout2 = []
            self.currReward = 0
            self.phase = 1
            self.agent.reset()

    ######### POST TRAINING #########
    def postTraining(self):     
        if self.mode == 'GET_STATE_DATA':
            self.data_to_txt(path = '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/' + 'SLOPE_PUSH' + '_state_data_GP3_over_edge.txt')
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