#! /usr/bin/env python

import argparse
from itertools import count
import scipy.optimize

import torch
from utils import *
from agent import Agent
from network import Network
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep

torch.utils.backcompat.broadcast_warning.enabled = True
torch.utils.backcompat.keepdim_warning.enabled = True

tau = .97
l2Reg = 1e-3
maxKL = 1e-2
damping = 1e-1
batch = 15000
logInterval = 1
num_inputs =10
num_actions = 10

#torch.set_default_tensor_type('torch.DoubleTensor')

class TRPOAgent(Agent):
    def __init__(self, params, name = "", task):
        super(TRPOAgent,self).__init__(params, name, task)
        self.policyNet = Network(params['actPars'], params['actTrain'])
        self.running_state = ZFilter((num_inputs,), clip=5)
        self.running_reward = ZFilter((1,), demean=False, clip=10)
        self.experience = Memory()
        task.initAgent(self)
        while(not self.stop):
            x = 1+1
        task.postTraining()
    
    def saveModel(self):
        torch.save(self.valueNet.state_dict(), "/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/TRPOCritic.txt")
        torch.save(self.policyNet.state_dict(), "/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/TRPOPolicy.txt")
        print("Network saved")

    def train(self):
        batch = self.experience.sample()
        self.update_params(batch)
    
    def store(self, prevS, prevA, r, s, a, failure):
        mask = 0 if failure == 1 else 1
        mask = 0 if failure == 1 else 1
        self.experience.push(self.prev['S'], prevA, mask, state, r)

    def update_params(self, batch):
        rewards = torch.Tensor(batch.reward)
        masks = torch.Tensor(batch.mask)
        actions = torch.Tensor(np.concatenate(batch.action, 0))
        states = torch.Tensor(batch.state)
        values = self.valueNet(Variable(states))

        returns = torch.Tensor(actions.size(0),1)
        deltas = torch.Tensor(actions.size(0),1)
        advantages = torch.Tensor(actions.size(0),1)

        prev_return = 0
        prev_value = 0
        prev_advantage = 0
        for i in reversed(range(rewards.size(0))):
            returns[i] = rewards[i] + self.discount * prev_return * masks[i]
            deltas[i] = rewards[i] + self.discount * prev_value * masks[i] - values.data[i]
            advantages[i] = deltas[i] + self.discount * tau * prev_advantage * masks[i]

            prev_return = returns[i, 0]
            prev_value = values.data[i, 0]
            prev_advantage = advantages[i, 0]

        targets = Variable(returns)

        # Original code uses the same LBFGS to optimize the value loss
        def get_value_loss(flat_params):
            set_flat_params_to(self.valueNet, torch.Tensor(flat_params))
            for param in self.valueNet.parameters():
                if param.grad is not None:
                    param.grad.data.fill_(0)

            values_ = self.valueNet(Variable(states))

            value_loss = (values_ - targets).pow(2).mean()

            # weight decay
            for param in self.valueNet.parameters():
                value_loss += param.pow(2).sum() * l2Reg
            value_loss.backward()
            return (value_loss.data.double().numpy(), get_flat_grad_from(self.valueNet).data.double().numpy())

        flat_params, _, opt_info = scipy.optimize.fmin_l_bfgs_b(get_value_loss, get_flat_params_from(self.valueNet).double().numpy(), maxiter=25)
        set_flat_params_to(self.valueNet, torch.Tensor(flat_params))

        advantages = (advantages - advantages.mean()) / advantages.std()

        output = self.policyNet(Variable(states)).view(-1, self.u_n * 2)
        action_means = output.narrow(1, 0, self.u_n)
        action_log_stds = output.narrow(1,self.u_n, self.u_n)
        action_stds = torch.exp(action_log_stds)

        fixed_log_prob = normal_log_density(Variable(actions), action_means, action_log_stds, action_stds).data.clone()

        def get_loss(volatile=False):
            if volatile:
                with torch.no_grad():
                    output = self.policyNet(Variable(states))
            else:
                output = self.policyNet(Variable(states))

            output = output.view(-1, self.u_n * 2)
            action_means = output.narrow(1, 0, self.u_n)
            action_log_stds = output.narrow(1,self.u_n, self.u_n)
            action_stds = torch.exp(action_log_stds)
                    
            log_prob = normal_log_density(Variable(actions), action_means, action_log_stds, action_stds)
            action_loss = -Variable(advantages) * torch.exp(log_prob - Variable(fixed_log_prob))
            return action_loss.mean()


        def get_kl():
            output  = self.policyNet(Variable(states))
            output = output.view(-1, self.u_n * 2)
            mean1 = output.narrow(1, 0, self.u_n)
            log_std1 = output.narrow(1,self.u_n, self.u_n)
            std1 = torch.exp(action_log_stds)
            
            mean0 = Variable(mean1.data)
            log_std0 = Variable(log_std1.data)
            std0 = Variable(std1.data)
            kl = log_std1 - log_std0 + (std0.pow(2) + (mean0 - mean1).pow(2)) / (2.0 * std1.pow(2)) - 0.5
            return kl.sum(1, keepdim=True)

        loss = trpo_step(self.policyNet, get_loss, get_kl, maxKL, damping)
        self.avgLoss += loss
        self.trainIt += 1