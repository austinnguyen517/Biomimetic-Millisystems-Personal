#! /usr/bin/env python

from task import Task, unitVector, dot, vector
from task import distance as dist
import numpy as np 
import math
import rospy
import torch 
import torch.nn as nn
import vrep
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from collections import OrderedDict
    
towards_x = np.array([-1, -1, 1, 1])
towards_y = np.array([1, -1, -1, 1])

'''TODO: Add buff for distance in x direction from contact point 
    TODO: Implement turning position set with four robots'''

class OmniBoxSlopeTask(Task):
    def __init__(self):
        super(OmniBoxSlopeTask, self).__init__()
        self.prev = {"S": None, "A": None}
        self.actionMap = {0: "STRAIGHT_BOX", 1: "HOME_CONTACT", 2: "CHANGE_ANGLE", 3: "BACK"} 
        self.gain = 200
        self.num_agents = 3
        '''
        self.currReward = 0
        self.rewards = []
        '''
        self.time = 0
        self.turn = 0 

        self.period = 5
        self.angle_threshold = .1
        self.counter = [0 for i in range(self.num_agents)]
        home_contact = [1.6 - i*(3.2)/(self.num_agents - 1) for i in range(self.num_agents)]
        turn_contact = [2.2] + [1.1 - i*(2.2)/(self.num_agents - 3) for i in range(self.num_agents - 2)] + [-2.2]
        self.y_contact = [home_contact, turn_contact, turn_contact, turn_contact]

        edges = [-.58, -.58, 0, -.75]
        mids = [-.58, -.58, -.58, -.58]
        self.x_contact = [edges, mids, mids, edges]
        self.phase = 0


    def extractInfo(self):
        # self.vTrain = self.agent.vTrain
        self.pubs = OrderedDict()
        for key in self.agents.keys():
            bot = self.agents[key]
            self.pubs[key] = rospy.Publisher(bot['pub'], String, queue_size = 1)
        # self.trainMode = self.agent.trainMode
        self.name = self.agent.name
        rospy.Subscriber(self.agents[self.name]['sub'], String, self.receiveState, queue_size = 1) 
 
    def sendAction(self, s):
        msg = String()
        changeAction = self.checkConditions(s, self.prev['A'])
        angles = self.getAngles(s)
        states = self.splitState(s)
        ret = []
        for i, name in enumerate(self.pubs.keys()):
            loc = "robot" + str(i)
            angle = angles[loc]
            state = states[loc]
            if changeAction[i]:
                self.counter[i] = self.period
                action = self.getNextAction(state, angle, i)
            else: 
                action = self.prev['A'][i]
            ret.append(action)
            action = self.getPrimitive(angle, self.actionMap[action], i)
            msg.data = vrep.simxPackFloats(action)
            self.pubs[name].publish(msg)
            self.counter[i] -= 1
        self.time += 1
        if self.time % np.inf == 0: 
            self.turn = -1 # turn left for now # np.random.randint(2) + 1 if self.turn == 0 else 0 
            self.phase = (self.phase + 1) % 4
            print("PHASE ",self.phase)
        print(ret, self.time)
        return ret
    
    def getNextAction(self, state, angle, i ):
        '''Order of priority: Box orientation (theta), Contact Point, Agent orientation'''
        theta = angle[3]
        tilt = angle[4]
        to_contact = dist(state[:2], np.array([self.x_contact[i][self.phase], self.y_contact[self.phase][i]]))
        bench = .25 if self.phase == 2 else .1
        if (i != 0 and i != self.num_agents - 1) or self.phase == 0 or self.phase == 1:
            if theta > self.angle_threshold: #you're ahead
                action = 3
            elif tilt or to_contact > bench: # far from your contact point 
                action = 1
            elif abs(angle[2]) > .2: # orientation incorrect
                action = 2
            else:
                action = 0  
        else:
            if tilt or to_contact > bench: # far from your contact point 
                action = 1
            elif theta > self.angle_threshold: #you're ahead
                action = 3
            elif abs(angle[2]) > .2: # orientation incorrect
                action = 2
            else:
                action = 0     
        return action
    
    
    def getPrimitive(self, angle, a, i): 
        # Given a local state s, action description a and distance of contact point from center of box dist
        alpha, beta, phi, box_ori, tilt = angle 
        damp = abs(self.y_contact[self.phase][i] / max(self.y_contact[self.phase]))

        if a == "STRAIGHT_BOX": 
            if self.phase == 2:
                direct = -1 if self.turn == 1 else 1
                orient = -damp*box_ori*(self.gain/5)*towards_x if (i != 0 and i != self.num_agents-1) else 0
                action = self.gain * direct*towards_y + orient + towards_x * self.gain/4 
            else:
                action = (self.gain - damp*box_ori * (self.gain/2))*towards_x
            angle_fix = np.array([self.gain/4 * np.sign(phi) for i in range(4)])
            action = (action + angle_fix).tolist()
        if a == "HOME_CONTACT": 
            direction = vector(beta)
            p = direction / np.sum(np.abs(direction))
            hover = towards_x / 6 if (self.phase == 2 and i != 0 and i != self.num_agents-1) else 0
            action = (p[0] * towards_y + p[1] * towards_x) + hover
            action = (1.5*self.gain*action).tolist()
        if a == "CHANGE_ANGLE":
            action = [self.gain * np.sign(phi) for i in range(4)]
        if a == "BACK":
            action = [self.gain / 4, self.gain / 4, -self.gain /4 , -self.gain/4]
        return action
    
    def splitState(self, s):
        s = np.array(s).ravel()
        states = {}
        for i in range(self.num_agents):
            start = i * 3
            end = (i + 1) * 3
            key = "robot" + str(i)
            states[key] = s[start: end]
        states["box"] = s[self.num_agents * 3:]
        return states
    
    def getAngles(self, s):
        states = self.splitState(s)
        box = states['box']
        box_ori = box[2]

        angles = {}
        if self.phase == 1 or self.phase == 3: # these are transitioning phases
            reached = True 
            for i in range(self.num_agents):
                to_contact = dist(states["robot" + str(i)][:2], np.array([self.x_contact[i][self.phase], self.y_contact[self.phase][i]]))
                if to_contact > .1:
                    reached = False
            if reached:
                self.phase = (self.phase + 1) % 4
                print('PHASE',self.phase)
        for i in range(self.num_agents):
            curr = states["robot" + str(i)]
            ori = curr[2]
            pos = curr[:2]

            # Calculating angle for STRAIGHT_BOX and CHANGE_ANGLE
            phi = ori - box_ori 
            direction = -1 if self.y_contact[self.phase][i] > 0 else 1
            # Special case: if you're smack in the middle, you should go back if either side is uneven
            direction = np.sign(box_ori) if self.y_contact[self.phase][i] == 0 else direction
            

            goal_relative_to_box = np.array([self.x_contact[i][self.phase], self.y_contact[self.phase][i]])
            phi_trans = phi - np.pi/2
            phi_trans = phi_trans + 2*np.pi if phi_trans < -np.pi else phi_trans
            goal = goal_relative_to_box - pos # this is the vector from current position to contact point all relative to box
            front_v = vector(phi_trans)
            right_v = vector(phi_trans - np.pi/2)

            # Calculating angles for HOME_CONTACT
            relative_y = -dot(unitVector(goal), right_v) # negative for convention
            relative_x = dot(unitVector(goal), front_v)
            buff = (-np.pi if relative_y < 0 else np.pi) if relative_x < 0 else 0 # since we want to map -pi to pi
            alpha = np.arctan(relative_y/relative_x) + buff 
            beta = -np.pi - alpha if alpha < 0 else np.pi - alpha 
            contact_angles = (alpha, beta)

            # Boolean to determine need for translation adjustment ie. if box is too far in, tilt it towards you
            ratio = abs(goal[1]/goal[0]) # ratio  horizontal and vertical distance from contact point 
            tilt = True if ratio > .8 and np.sign(goal[1] * self.y_contact[self.phase][i]) < 0 and abs(goal[0]) < .4 else False 

            angles['robot' + str(i)] = (alpha, beta, phi, direction*box_ori, tilt)
        
        return angles

    def checkConditions(self, full_state, a):
        # given self.prev['A'] and state (unraveled already), check that we've sufficiently executed primitive
        if a == None:
            return [True for i in range(self.num_agents)]
        fill = []
        states = self.splitState(full_state)
        prev_states = self.splitState(self.prev['S'])
        angles = self.getAngles(full_state)
        box = states["box"]
        prevBox = prev_states['box']
        for i in range(self.num_agents):
            k = "robot" + str(i)
            angle = angles[k]
            theta = angle[3]
            act = self.actionMap[a[i]]
            curr = states[k]
            if act == "STRAIGHT_BOX":
                fill.append(self.timeOut(i))
            if act == "HOME_CONTACT":
                fill.append(self.timeOut(i) or self.fallingBehind(theta) or dist(curr[:2], np.array([-.58, self.y_contact[self.phase][i]])) < .3)
            if act == "CHANGE_ANGLE":
                fill.append(self.timeOut(i) or self.fallingBehind(theta) or abs(box[2] - curr[2]) < .05)
            if act == "BACK":
                fill.append(self.timeOut(i))
        return fill
    
    def timeOut(self, i):
        return self.counter[i] == 0

    def fallingBehind(self, theta):
        return theta < -self.angle_threshold

    def getAux(self, pos, prevPos, blockPos, prevBlock, ori, prevOri, phase, blockOri):
        return 

    def rewardFunction(self, s, a):
        s = s.ravel()
        prevS = self.prev["S"].ravel()
        # THIS IS A TEST. NO REWARD
        return (-.1, 0)# self.getAux(pos, prevPos, blockPos, prevBlock, ori, prevOri, self.phase, blockOri) 
    
    def isValidAction(self, s, a):
        return not self.checkConditions(s.ravel(), a)

    def receiveState(self, msg):    
        floats = vrep.simxUnpackFloats(msg.data)
        fail = floats[-1]
        restart = 0

        s = (np.array(floats)).reshape(1,-1)

        a = (self.sendAction(s))
        # if type(self.prev["S"]) == np.ndarray:
        #     r, restart = self.rewardFunction(s, self.prev["A"])
        #     r = r if self.isValidAction(self.prev['S'], self.prev['A']) else -.01
        #     if r == 5:
        #         print(" #### Phase ", self.phase, "  Complete!")
        #         self.phase += 1
        #     self.agent.store(self.prev['S'], self.prev["A"], np.array([r]).reshape(1, -1), s, a, restart)
        #     self.currReward += r

       # if self.trainMode:
           # loss = self.agent.train()

        self.prev["S"] = s
        self.prev["A"] = a
        # else:
            # a = self.sendAction(s)
            # SPECIAL CASE: since we execute one primitive for multiple time steps (with intermittent control updates), we need to store transitions/rewards when the agent fails out or succeeds
            # if type(self.prev["S"]) == np.ndarray:
            #     r, restart = self.rewardFunction(s, self.prev["A"])
            #     if restart:
            #         if r > 0: # we assume failure has rewards < 0
            #             print(' #### Success!')
            #         else:
            #            print(' #### Dropped')
            #         self.agent.store(self.prev['S'], self.prev["A"], np.array([r]).reshape(1, -1), s, a, restart)
            #         self.currReward += r
        self.restartProtocol(restart or fail)

        return 
    
    def restartProtocol(self, restart):
        if restart == 1:    
            print('Results:     Cumulative Reward: ', self.currReward, '    Steps: ', self.agent.totalSteps)
            print("")
            for k in self.prev.keys():
                self.prev[k] = None
            self.goal = 0
            if self.currReward != 0:
                self.rewards.append(self.currReward)
            self.currReward = 0
            self.agent.reset()

    ######### POST TRAINING #########
    def postTraining(self):
        #valueOnly = True if self.a == "argmax" else False
        self.plotRewards()
        self.plotLoss(False, 'Loss Over Iterations w/ Moving Average', "Actor Loss over Iterations w/ Moving Average")
        #self.agent.saveModel()
    
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