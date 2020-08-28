#! /usr/bin/env python

import numpy as np 
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import math 
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3
import vrep
import matplotlib.pyplot as plt
import sys
import time

from Networks.model_network import Model
from Tasks.task import distance as dist
from agent import Agent
from Tasks.hierarchyTask import HierarchyTask
from doubleQ import DoubleQ
from Buffers.CounterFactualBuffer import Memory
from scipy.stats import multivariate_normal
from sklearn.mixture import GaussianMixture
import pickle

class Hierarchical_MBRL(Agent):
    def __init__(self, params, name, task, load_path=None):
        super(Hierarchical_MBRL, self).__init__(params, name, task)
        if self.trainMode:
            self.policy = DoubleQ(params['doubleQ_params'], name, task, load_path=1) # IMPORTNAT NOTE: make sure to initialized DoubleQ first. 
            # Otherwise, the task will reference DoubleQ as the agent...not this class (Hierarchical_MBRL)
            self.model = Model(self.vPars, self.vTrain, task)
        else:
            self.policy = DoubleQ(params['doubleQ_params'], name, task, load_path='/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/hierarchical_q_policy2.txt') 
            self.model = Model(self.vPars, self.vTrain, task)
            # self.model.load_state_dict(torch.load('/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Pure_Q/Combination_2/model_differential2.txt'))
            # paths = ['/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model.txt']
            # self.valueNet.load_state_dict(torch.load(load_path))

        self.u_n = self.vPars['u_n']
        self.explore = self.vTrain['explore']

        self.num_sequences = 50
        self.length_sequences = 10
        self.controller = HierarchyTask()

        """
        Pure_MPC: Training model and only using MPC for policy.                         Testing: Uses MPC only
        Pure_MFRL: Training Q hierarchical policy without use of model                  Testing: Uses Q Policy only 
        AIDED_MFRL: Training using model to aid control selection for Q policy          Testing: Uses Q Policy only
        """
        self.method = 'Pure_MFRL'
        self.policy_action_selection = 'DETERM' #'PROB'#
        self.testing_to_record_progress = True
        self.epochs = 200

        self.counter = 0
        self.train_every_steps = 1

        self.gmm_counter = 0
        self.gmm_period = 1
        self.weight_towards_model = .7
        self.success_states = np.zeros((1,9))

        torch.set_num_threads(2)


        self.loss = []
        self.validation_loss = []
        self.avgLoss = 0

        task.initAgent(self)
        self.preTrain = self.vTrain['pretrain']
        if self.preTrain:
            self.train_model_beforehand()
            self.task.plotLoss()
    
        if not load_path:
            while(not self.stop):
                x = 1+1
            data = self.model.transitions
            self.save_data(data)
            task.postTraining()
    
    def successful_append(self, rollout_list):
        end_state = rollout_list[-1]
        self.success_states = np.vstack((self.success_states, end_state))
        

    def saveModel(self):
        torch.save(self.model.state_dict(), '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/model_differential2.txt')
        self.policy.saveModel()
        pass
    
    def save_data(self, data):
        path = '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/model_data_elevated_push_hole_differential_2.txt'
        with open(path, "wb") as fp:  
            pickle.dump(data, fp)
        return 
    
    def train_model_beforehand(self):
        path = '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Pure_Q/Combination_2/model_data_elevated_push_hole_differential_2.txt'
        with open(path, "rb") as fp:  
            data = pickle.load(fp)
        
        sets = np.split(data, [int(data.shape[0] * .8), data.shape[0]])
        training = sets[0]
        validation = sets[1]

        s = training[:1000, :10]
        a = training[:1000, 10]
        s_prime = training[:1000, -9:]
        self.model.store(s, a, s_prime)
        #if self.model.one_hot_flag:
        #    self.model.store(data[:, :10], data[:, 10], data[:, -9:])
        #else:
        #    self.model.store(data[:, :10], data[:, 10:12], data[:, -9:])
        for i in range(500):
            training_loss = self.model.train()
            self.loss.append(training_loss)
            self.model.store(data[i + 2000: i + 2001, :10], data[i + 2000: i + 2001, 10], data[i + 2000: i + 2001, -9:])
            
            predictions = self.model.predict(validation[:, :10], validation[:, 10])
            square_delta = torch.pow(predictions - torch.FloatTensor(validation[:, -9:]), 2)
            square_delta = torch.sum(square_delta, axis=1)
            validation_loss = square_delta.mean()
            self.validation_loss.append(validation_loss)
            if i % 500 == 0:
                print('Iteration: ', i, '   Training loss:', training_loss, '   Validation loss:', validation_loss)
        plt.plot(range(len(self.loss)), self.loss)
        plt.plot(range(len(self.validation_loss)), self.validation_loss)
        plt.show()
        sys.exit(0)
        return 
    
    def store(self, s, a, r, sprime, aprime, done, action_index):
        if not self.testing_to_record_progress:
            self.counter += 1
            s = self.concatenate_identifier(s)
            a = np.array([a]) if type(a) == int else a
            self.model.store(s, a, sprime)
            sprime = self.concatenate_identifier(sprime)
            self.policy.store(s, action_index, np.array([r]).reshape(1, -1), sprime, aprime, done)
            print(' Experience length: ', max(len(self.policy.exp), len(self.model.transitions)))

    def concatenate_identifier(self, s):
        return np.hstack((s, np.repeat(1, s.shape[0]).reshape(-1,1)))
    
    def get_action(self, s):
        self.task.stop_moving()
        """
        Pure_MPC: Training model and only using MPC for policy.                         Testing: Uses MPC only
        Pure_MFRL: Training Q hierarchical policy without use of model                  Testing: Uses Q Policy only
        AIDED_MFRL: Training using model to aid control selection for Q policy      Testing: Uses Q Policy only
        """
        if not self.testing_to_record_progress:
            self.explore = max(.1, self.explore * .9997) # NOTE: was .9996 before
        if self.method == 'Pure_MPC':
            return self.solo_MPC_model_return(s, testing_time=self.testing_to_record_progress)
        if self.method == 'Pure_MFRL':
            return self.return_q_policy_action_index(s, testing_time=self.testing_to_record_progress)
        if self.method == 'AIDED_MFRL':
            if not self.testing_to_record_progress:
                return self.gaussian_explore_q(s, testing_time = self.testing_to_record_progress)
            else:
                return self.return_q_policy_action_index(s, testing_time=self.testing_to_record_progress)

    def return_q_policy_action_index(self, s, testing_time):
        i = np.random.random() 
        print('')
        if i < self.explore and not testing_time and self.trainMode:
            return np.random.randint(self.u_n)
        return self.policy.get_action(self.concatenate_identifier(s), testing_time, probabilistic=(self.policy_action_selection == 'PROB'))
    
    def solo_MPC_model_return(self, s, testing_time):
        actions = np.random.choice(self.u_n, (self.num_sequences, self.length_sequences))
        states = np.repeat(s, self.num_sequences, axis=0)
        completely_random = np.random.random()
        if completely_random < self.explore and not testing_time:
            return np.random.randint(self.u_n)
        
        smallest_cost = np.zeros((self.num_sequences, 1)) + np.inf
        for i in range(self.length_sequences):
            no_cat = states
            states = self.concatenate_identifier(states)
            if i == 0:
                starting_states = no_cat
            action_indices = actions[:, i]
            if self.model.one_hot_flag:
                states = self.model.predict(states, action_indices.ravel()).detach().numpy()
            else:
                action_differential = np.zeros((self.num_sequences, 2))
                for j in range(self.num_sequences):
                    self.controller.goal = no_cat[j, :2]
                    diff_drive = self.controller.getPrimitive(self.controller.feature_2_task_state(no_cat[j, :].ravel()), self.task.action_map[action_indices[j]])
                    action_differential[j] = diff_drive
                states = self.model.predict(states, action_differential).detach().numpy() #predict already takes into account delta
            # TODO: This is incorect. Should maxiize over the rewards!!!
            cost = self.get_mpc_reward_of_trajectory(starting_states, states)
            smallest_cost = np.minimum(cost, smallest_cost)

        index_best_sequence = np.argmin(smallest_cost)

        next_action_index = actions[index_best_sequence, 0]
        return next_action_index
    
    def gaussian_explore_q(self, s, testing_time):
        exploration = np.random.random()
        if exploration < self.explore and not testing_time:
            if len(self.model.transitions) > 400 and self.success_states.shape[0] >= 2: 
                curr_size = self.success_states.shape[0] - 1
                if curr_size == 0:
                    return np.random.randint(self.u_n)
                sample_size = min(1000, curr_size) 
                success_states = self.success_states[1:, :]
                past_states = success_states[-sample_size:, :]

                actions = [i for i in range(self.u_n)]
                states = np.repeat(s, self.u_n, axis=0)
                states = self.concatenate_identifier(states)
                next_states = self.model.predict(states, actions).detach().numpy()
                
                """                 
                if self.gmm_counter % self.gmm_period == 0:
                    self.gmm = GaussianMixture(n_components=1) 
                    self.gmm.fit(past_states)

                log_prob = self.gmm.score_samples(next_states)
                log_prob = log_prob - np.min(log_prob)
                log_prob = np.exp(log_prob)

                probs = log_prob / np.sum(log_prob)
                print('probs: ', probs)
                self.gmm_counter += 1
                self.success_states = self.success_states[:5000, :]"""
                    
                """mean = np.mean(past_states, axis=0)
                centered = past_states - mean
                covariance = np.matmul(centered.T, centered) / (past_states.shape[0])
                covariance = covariance + np.eye(covariance.shape[0]) * 1e-2
                pdf_generator = lambda x: multivariate_normal.pdf(x, mean=mean, cov=covariance)

                pdf_of_next_states = np.apply_along_axis(pdf_generator, 1, next_states)
                pdf_of_next_states = pdf_of_next_states - np.min(pdf_of_next_states)
                probs = np.exp(pdf_of_next_states)
                probs = probs / np.sum(probs)"""

                avg_end_state = np.mean(past_states, axis=0)
                probs = np.array([np.exp(-7*dist(avg_end_state, state)) for state in next_states])
                probs = probs / np.sum(probs)

                uniform = np.ones((1, self.u_n)) / self.u_n
                q_lookahead = np.max(self.policy.get_q(self.concatenate_identifier(next_states)).detach().numpy(), axis=1)
                q_lookahead = np.exp(q_lookahead)
                q_lookahead = q_lookahead / np.sum(q_lookahead)

                probs = self.weight_towards_model * probs + (1 - self.weight_towards_model) * uniform # it's qlookahead rn
                self.weight_towards_model = max(.1, self.weight_towards_model * .999)
                probs = probs.flatten()
                probs = probs / np.sum(probs) # just in case

                print('weight:', self.weight_towards_model, ' probs:', probs)
                choice = np.random.choice(self.u_n, p=probs)
                return actions[choice]
            else:
                return np.random.randint(self.u_n) 
        else:
            return self.policy.get_action(self.concatenate_identifier(s), testing_time, probabilistic=(self.policy_action_selection == 'PROB'))
    
    def get_indices_of_candidates(self, q_values):
        top = 2
        prev_indices_of_top_q = np.array([np.argmax(q_values)])
        while top <= q_values.size:
            top_q_values = sorted(q_values)[-top:]
            indices_of_top_q = q_values.argsort()[-top:]
            probs = np.exp(top_q_values)
            probs = probs / np.sum(probs)
            relative_entropy = self.relative_entropy(probs)

            try:
                assert prev_indices_of_top_q.size + 1 == probs.size
            except AssertionError:
                print(' invalid sizes: ', prev_indices_of_top_q.size, probs.size)

            if relative_entropy < .97:
                print(prev_indices_of_top_q, probs, relative_entropy)
                return prev_indices_of_top_q

            prev_indices_of_top_q = indices_of_top_q
            top += 1
        print(indices_of_top_q, probs, relative_entropy)
        return indices_of_top_q
    
    def relative_entropy(self, list_probs):
        base = list_probs.size
        entropy = np.sum(list_probs * (np.log(list_probs) / np.log(base)))
        uniform = np.ones(list_probs.size) / base
        max_entropy = np.sum(uniform * (np.log(uniform) / np.log(base)))
        return entropy / max_entropy

    
    def get_mpc_reward_of_trajectory(self, starting_states, states):
        box = states[:, :3]
        goal = states[:, 5:8]
        dist_box_to_goal = np.sqrt(np.sum(np.square(box - goal), axis=1))
        dist_box_to_robot = np.sqrt(np.sum(np.square(box), axis=1))
        orientation = states[:, 8]

        prev_dist_box_to_goal = np.sqrt(np.sum(np.square(starting_states[:, :3] - starting_states[:, 5:8]), axis=1))
        #return dist_box_to_goal * 2 + dist_box_to_robot + np.abs(orientation) * .25
        return prev_dist_box_to_goal - dist_box_to_goal 
    
    def train_model(self):
        if self.trainMode and self.method == 'AIDED_MFRL':
            print('Started Training Model')
            for i in range(self.epochs):
                loss = self.model.train()
                self.loss.append(loss)
            print('Finished Training Model')
            plt.plot(range(len(self.loss)), self.loss)
            plt.savefig( '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/value_loss_update.jpg')


    def train(self, episode=0):
        if max(len(self.policy.exp), len(self.model.transitions)) >= 2 and not self.testing_to_record_progress and self.trainMode:
            if self.method == 'Pure_MFRL' or self.method == 'AIDED_MFRL':
                if self.counter % self.train_every_steps == 0:
                    loss = self.policy.train(override=True)  
            if self.method == 'Pure_MPC': 
                loss = self.model.train()
                self.loss.append(loss)
