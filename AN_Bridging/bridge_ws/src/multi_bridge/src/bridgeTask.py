#! /usr/bin/env python

from task import Task 
from utils import distance
import numpy as np 
import torch 
import torch.nn as nn 

class BridgeTask(Task):
    def __init__(self, action):
        super(bridgeTask, self).__init__(action)
        self.prev = {"S": None, "A": None, "D": None}
        self.actionMap = {0: (-3,-1), 1:(-1,-3), 2:(-3,-3), 3:(1,3), 4:(3,3), 5:(3,1), 6:(0,0)}
        self.tankRewards = {"velocity": [], "location": [], "relativeLocation": [], "orientation": [], "relativeOrientation":[] , "total": []}
        self.bridgeRewards = {"velocity": [], "location": [], "relativeLocation": [], "orientation": [], "relativeOrientation": [], "total":[]}
        self.tankRun = {"velocity": 0, "location": 0, "relativeLocation": 0, "orientation": 0, "relativeOrientation":0 , "total": 0}
        self.bridgeRun = {"velocity": 0, "location": 0, "relativeLocation": 0, "orientation": 0, "relativeOrientation": 0, "total":0}
    
        self.phases = [False, False, False] #bridge, across, pull
        self.valueLoss = []
        self.goal, self.avgLoss, self.trainIt = (0,0,0)

    
    ######### EXTRACT INFO ###########
    def extractInfo(self):
        self.name = self.agent.name
        self.vTrain = self.agent.vTrain
        self.w_loc = self.vTrain['alpha1']
        self.w_vel = self.vTrain['alpha2']
        self.w_ori = self.vTrain['alpha3']
        self.w_age = self.vTrain['lambda']
        self.u_n = self.agent.u_n
        self.agents = self.agent.agents
        self.state_n = self.agent.state_n
    
    ############# SEND ACTION #################
   def sendAction(self, s):
        if self.a == "argmax":
            q = self.valueNet.predict(s)
            i = np.random.random()
            if i < self.explore:
                index = np.random.randint(self.u_n)
            else:
                index = np.argmax(q.detach().numpy())
            tank, bridge = self.index_to_action(index)
            self.pubs['tanker'].publish(tank)
            self.pubs['bridger'].publish(bridge)
            return np.array([index]).reshape(1,-1), self.prev["A"] >= 50
        if self.a == "p_policy":
            output = self.policyNet(torch.FloatTensor(state))
            state = torch.from_numpy(state).unsqueeze(0)
            action_mean = output[:, :self.u_n]
            action_logstd = output[:, self.u_n:]
            action_std = torch.exp(action_logstd)
            action = (torch.normal(action_mean, action_std).detach().numpy()).ravel()
            if action[self.u_n - 2]>0: #rope 
                rope = 1 if action[self.u_n - 1] > 0 else 2
            if action[self.u_n - 2]<=0: #neutral
                rope = -1 if action[self.u_n - 1] >0 else 0
            tankmsg = Vector3()
            bridgemsg = Vector3()

            tankmsg.x = action[0]
            tankmsg.y = action[1]
            tankmsg.z = rope 
            bridgemsg.x = action[2]
            bridgemsg.y = action[3]
            self.pubs['tanker'].publish(tankmsg)
            self.pubs['bridger'].publish(bridgemsg)
            return action.reshape(1,-1), self.prev["A"[self.u_n -2]] > 0
        return 

    def index_to_action(self, index):
        tankmsg = Vector3()
        bridgemsg = Vector3()
        if index >= 49:
            tankmsg.x = 0
            tankmsg.y = 0
            tankmsg.z = index - 49
            bridgemsg.x = 0
            bridgemsg.y = 0
        else:
            tank = self.actionMap[index // 7]
            bridge = self.actionMap[index % 7]
            tankmsg.x = tank[0]
            tankmsg.y = tank[1]
            tankmsg.z = -1
            bridgemsg.x  = bridge[0]
            bridgemsg.y = bridge[1]
        return (tankmsg, bridgemsg)

    ########## REWARD FUNCTION ###############
   def rewardFunction(self, s, usedRope): 
        s = s.ravel()
        
        r = self.checkPhase(s)
        if r != 0:
            return r

        prevS = self.prev["S"].ravel()
        pos = np.array(s[:3])
        prevPos = np.array(prevS[:3])
        ori = s[5]
        prevOri = prevS[5]


        R_loc, R_vel, R_ori = self.agentReward(pos, prevPos, ori, prevOri)
        R_own = R_loc + R_vel + R_ori

        self.tankRun["location"] += R_loc 
        self.tankRun['velocity'] += R_vel
        self.tankRun["orientation"] += R_ori 

        
        R_age = 0
        i = 0
        for key in self.agents.keys():
            botPars = self.agents[key]
            oPos = s[i: i + 3]
            prevOPos = np.array(prevS[i: i+3])
            R_loc, R_vel, R_ori = self.agentReward(oPos, prevOPos, s[i + 5], prevS[i+5])

            R_agents = R_loc + R_vel + R_ori
            self.bridgeRun["location"] += R_loc 
            self.bridgeRun['velocity'] += R_vel
            self.bridgeRun["orientation"] += R_ori 

            deltaX = abs(oPos[0]) - abs(pos[0])
            deltaY = abs(oPos[1]) - abs(pos[1])
            relativeLoc = self.w_loc * (self.prev["D"][0] - deltaX + self.prev["D"][1] - deltaY)
            R_age += relativeLoc  #we don't like it when the robots are far apart
            relativeOri = 0#abs(ori - s[i + 3]) * self.w_ori
            R_age -= relativeOri

            i += botPars["n"]
        

        self.tankRun["relativeLocation"] -= relativeLoc 
        self.bridgeRun["relativeLocation"] -= relativeLoc 
        self.tankRun["relativeOrientation"] -= relativeOri 
        self.bridgeRun["relativeOrientation"] -= relativeOri


        R_rope = -3 if usedRope else 0
        reward = R_own + self.w_age * R_age + R_rope

        self.tankRun['total'] += reward
        self.bridgeRun['total'] += reward
        return reward

   def checkPhase(self, state):
        pos0 = np.array(state[:3])
        pos1 = np.array(state[self.agents[self.name]["n"]: self.agents[self.name]["n"] + 3])
        if distance(pos0, self.goal) <.03 or distance(pos1, self.goal) <.03:
            return 50
        if pos0[2] <= .1 or pos1[2] <= .1:
            return -10
        if not self.phases[0] and pos1[0] > -.52: #achieved BRIDGE phase. Adjust accordingly
            self.phases[0] = True 
            return 15
        if not self.phases[1] and pos0[0] > -.4: #achieved CROSS phase. Adjust accordingly
            self.phases[1] = True 
            return 20
        if not self.phases[2] and pos1[0] > -.35: #achieved PULL phase. Adjust accordingly
            self.phases[2] = True
            return 25
        return 0

    def agentReward(self, pos, prevPos, ori, prevOri):
        '''Calculate distance from the goal location'''
        deltas = self.goal - pos
        scaleX = np.exp(5 - 5*abs(deltas[0]))
        scaleY = np.exp(5*abs(deltas[1]))
        R_loc = deltas[0] * 2 * self.w_loc

        '''Add the movement in x and subtract the movement in y'''
        deltas = pos - prevPos
        deltaX = 5 * deltas[0] * self.w_vel #* scaleX #positive is good 
        deltaY = abs(deltas[1]) * self.w_vel# * scaleY
        R_vel = (deltaX - deltaY)


        '''Calculate the delta of angle towards the goal. Subtract reward '''
        delta = np.abs(prevOri) - np.abs(ori)
        R_ori = delta *self.w_ori - (np.abs(ori) * .1)

        return (R_loc, R_vel, R_ori)

    ######### RECEIVE STATE #############
    def receiveState(self, msg):
        floats = vrep.simxUnpackFloats(msg.data)
        self.goal = np.array(floats[-4:-1])
        fail= floats[-1]
        s = (np.array(floats[:self.state_n])).reshape(1,-1)

        i, useRope = (self.sendAction(s))
        if type(self.prev["S"]) == np.ndarray:
            penalty = 1 if useRope else 0
            r = np.array(self.rewardFunction(s, penalty)).reshape(1,-1)
            self.store(self.prev['S'], self.prev["A"], r, s, i, failure)
            self.dataSize += 1
        self.prev["S"] = s
        self.prev["A"] = i
        s = s.ravel()
        self.prev["D"] = [abs(s[0]) - abs(s[4]), abs(s[1]) - abs(s[5]), abs(s[3]) - abs(s[7])]
        if self.trainMode:
            self.train()
        self.restartProtocol(fail)
        return 

    ######### RESTART PROTOCOL ##########
    def restartProtocol(self, restart = 0):
        if restart == 1:
            for k in self.prev.keys():
                self.prev[k] = None
            self.goal = 0
            if self.trainIt > 0:
                self.valueLoss.append((self.avgLoss)/self.trainIt)
            self.avgLoss = 0    
            self.trainIt = 0
            for k in self.tankRun.keys():
                self.tankRewards[k].append(self.tankRun[k])
                self.bridgeRewards[k].append(self.bridgeRun[k])
                self.tankRun[k] = 0
                self.bridgeRun[k] = 0

    ######### POST TRAINING #########
    def postTraining(self):
        self.plotLoss(True, "Centralized Q Networks: Q Value Loss over Iterations")
        self.plotRewards()
        self.saveModel()
    
    def plotLoss(self, valueOnly = False, title1 = "Critic Loss over Iterations", title2 = "Actor Loss over Iterations"):
        plt.plot(range(len(self.valueLoss)), self.valueLoss)
        plt.title(title1)
        plt.show()
        if not valueOnly:
            plt.plot(range(len(self.actorLoss)), self.actorLoss)
            plt.title(title2)
            plt.show()
    
    def plotRewards(self):
        for k in self.tankRewards.keys():
            plt.plot(range(len(self.tankRewards[k])), self.tankRewards[k], label = k)
        plt.title("Tank Rewards over Episodes")
        plt.legend()
        plt.show()
        for k in self.bridgeRewards.keys():
            plt.plot(range(len(self.bridgeRewards[k])), self.bridgeRewards[k], label = k)
        plt.title("Bridge Rewards over Episodes")
        plt.legend()
        plt.show()


