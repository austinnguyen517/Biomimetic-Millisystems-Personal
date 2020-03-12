import numpy as np 
import pytorch as torch
import pytorch.nn as nn
import math 
from network import Network
import rospy
from std_msgs.msg import String

# Collaborative agent in multi-agent framework. Initiates a class that contains:
    # actor network
    # critic network
    # actor network copy for KL divergence calculations

#Supports MADDPG with TRPO inspiration and optimizations in better scaling towards larger multiagent systems and stability

#TODO: ensure that the frequency in which we make exchanges is even ie. the state action reward state tuples are consistent in terms of timing 

actPars = {'state_n': ,
            'u_n':, 
            'output_n': ,
            'hidden': ,
            'depth': ,
            'activation': ,
            'preprocess':,
            'postprocess'
            'epochs':,
            'loss_fnc:'
            }
criticPars = {'prob': False,
            'sigma':, 
            'state_n':  ,
            'output_n': ,
            'hidden': ,
            'depth':,
            'activation':,
            'preprocess': ,
            'postprocess':,
            'epochs':,
            'loss_fnc': ,
            'discrete': 
            }
trainPars {'alpha1': ,
            'alpha2': ,
            'alpha3': ,
            'lambda': ,
            'batch': ,
            'horizon': ,
            'buffer': ,
            'explore': ,
            'lr': ,
            }
stateSub = "state"


class agent(nn.Module):
    def _init(self, actorParams, criticParams, atrainParams, ctrainParams):
        self.actor = None 
        self.critic = None 
        self.prevActor = None
        self.discount = gamma 

        self.critic_var = criticParams['sigma']
        self.weight_loc = trainParams['alpha1']
        self.weight_phase = trainparams['alpha2']
        self.weight_angle = trainParams['alpha3']
        self.weight_agents = trainparams['alpha4']
        self.base_reward = trainParams['lambda']
        self.batch_size = trainParams['batch']
        self.horizon = trainParams['horizon']
        self.prob = criticParams['prob']

        self.state_n = criticParams['state_n']
        self.u_n = actorParams['u_n']
        replayFeatures = 2*self.state_n + self.u_n
        self.expSize = trainParams['buffer']
        self.experience = np.zeros(self.expSize, replayFeatures)

        self.exploration = trainParams['explore']
        self.dataSize = 0 #number of data tuples we have accumulated so far

        self.actor = Network(actorParams, aTrainParams)
        self.critic = Network(criticParams, cTrainParams)

        self.prevState = None
        self.prevAction = None 

        rospy.Subscriber(stateSub, String, self.receiveState, queue_size = 1) 
        self.aPub = rospy.Publisher(actionPub, String, queue_size = 1)

    def receiveState(self, message):
        #get new state from v-rep using ROS. put that into the experience
        floats = vrep.simxUnpackFloats(message.data)
        #when you receive a state, send the action as well and save it in experience replay
        #that way, we maintain some form of uniformity in the frequency of sampling
        state = floats[:dummy]
        action = self.sendAction(state)
        r = rewardFunction(newState)
        if self.prevState != None:
            self.experience[self.dataSize] = np.hstack((prevState, self.prevAction, r, state))
            self.dataSize += 1
        self.prevstate = state 
        self.prevAction = action 
        return 
        
    
    def sendAction(self, state):
        out = self.actor.predict(state)
        mean = out[:self.state_n] 
        #EXPLORATION
        if self.prob:
            var = out[self.state_n:]
            action = self.sample(mean, var) + np.random.normal(0, self.exploration, self.u_n)
            action += np.random.normal(0, self.exploration, self.u_n)
        else:
            i = np.random.random()
            if i < self.exploration:
                action = np.random.randint(0, self.u_n)
            else:
                action = np.argmax(mean)
        self.aPub.publish(action)
        return action

    
    def sample(self, mean, var):
        return np.random.normal(mean, var)

    
    def rewardFunction(state, observations):
        #given a state and observations of the other n-1 agents, calculate the reward
        #state: 1 element: height of end of ravine to beginning
        #3 element: relative coordinates from the current fixed robot (could be 0,0,0)
        #3 elements: bit values for which phase of the process we are in
        #1 element: bit value condition of rope (0 or 1 for attached/detached)
        #3 elements: angles relative to normal direction for pitch, roll, yaw each robot 

        #observations are passed in as true state of other agents with Gaussian noise added relative to distance from agent 

        reward = 0
        reward += (state[1] + state[2] + state[3])*self.weight_loc #assume the coordinate frame is positive in direction towards end of bridge
        reward += (state[4] + 2*state[5] + 4*state[6])*self.weight_phase + self.base_reward
        reward += (state[8] + state[9] + state[10]) * self.weight_angle 
        for state in observations:
            state = observations[i]
            curr = 0
            curr += (state[1] + state[2] + state[3])*self.weight_loc #assume the coordinate frame is positive in direction towards end of bridge
            curr += (state[4] + 2*state[5] + 4*state[6])*self.weight_phase + self.base_reward
            curr += (state[8] + state[9] + state[10]) * self.weight_angle 
            reward += curr * self.weight_agents 

        return reward

        
    def train(self):
        #TODO: calculate the targets for the value function
        #TODO: Gaussiaan noise into network inputs
        #TODO: keep track of training and testing losses
        choices = np.random.choice(0, min(self.expSize, self.dataSize), batch_size) 
        data = self.experience[choices]
        targets = #calculate the VALUE of states!
        self.critic.train(data[:, :self.state_n], targets)
        if self.experience.size() >= self.horizon:
            index = np.random.randint(0, self.experience.size() - self.horizon)
            data = self.experience[index: index + self.horizon]
            states = data[:,:self.state_n]
            statePrime = data[:, -self.state_n:]
            valueS = self.critic.predict(data[:, states)
            valueSPrime = self.critic.predict(statePrime)
            advantage = data[:, self.state_n + self.u_n] + self.discount*valueSPrime + valueS #TODO: make sure properly gets reward value 
            '''We have the reward, state, next State and action associated. Calculate neew probability of doing that action. doesn't change the fact that this action will yield same results'''
            actions = self.experience[:, self.state_n: self.state_n + self.u_n]
            self.actor.train(states, actions, advantage)
        
