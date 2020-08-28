#! /usr/bin/env python


import numpy as np 
import torch 
import torch.nn as nn
import math 
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs. msg import Vector3
from collections import OrderedDict

'''from Algs.QSARSA import CentralQSarsa
from Algs.doubleQ import DoubleQ
from Algs.trpo import TRPOAgent
from Algs.TD3 import Twin_DDPG
from Algs.SoftActorCritic import SAC'''
from Algs.FuN import Feudal
from Algs.HIRO import HIRO
from Tasks.boxTask import BoxTask

NAME = 'bot'

algs = {
    3: "CENTRAL_Q", #REMINDER: if choosing 3, make sure to only run the tankAgent.py in the launch file
    4: "CENTRAL_Q_SARSA", #REMINDER: same as above
    5: "CENTRAL_TRPO",
    6: "CENTRAL_DDPG",
    7: 'SAC',
    8: 'FEUDAL',
    9: 'HIRO'
}
ALGORITHM = 9
description = algs[ALGORITHM]
rospy.init_node('Dummy', anonymous = True)

if description == "CENTRAL_Q":
    agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot": {"sub": "/state", "pub": "/action"} #joint action space
            })

    valPars = {
                'neurons':      (13, 256, 256, 6),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
                'trainMode':    True,
                'load':         False, 
                }             
    valTrain = {
                'batch':        128, 
                'lr':           3e-4, 
                'gamma':        .99,
                'explore': .4, 
                'double': True,
                }
    params = {"valPars": valPars, "valTrain": valTrain, "agents": agents}
    tanker = CentralQ(params, NAME, BoxTask())

if description == "CENTRAL_Q_SARSA":

    agents = {
                "bot": {"sub": "/state", "pub": "/action"}
            }
    valPars = {
                'neurons':      (13, 256, 256, 6),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
                'trainMode':    True,
                'load':         False,
                }             
    valTrain = {

                'batch':        1, 
                'lr':           3e-4, 
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'buffer':       1,
                'explore':      .3,
                'gamma':        .99,
                'baseExplore': .1,
                'decay': .75,
                'step': 75,
                'QWeight': 0,
                'gamma': .99
                }
    params = { "valPars": valPars, "valTrain": valTrain, "agents": agents}
    tanker = CentralQSarsa(params, NAME, BoxTask("argmax"))

if description == "CENTRAL_TRPO":
    agents = {
                #ensure order matches ros messages
                "bot": {"sub": "/state", "pub": "/action"}
            }
    actPars = {

                'neurons':      (13, 256, 256, 2),
                'act':          ['F.relu', 'F.relu'],
                'mean_range':   2,
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
            }
    valPars = {
                'neurons':      (13, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
                'trainMode':    True,
                'load':         False,
                }
    actTrain = { 
                'lr': 1e-5,
                }

    valTrain = {
                'batch': 64,
                'lr': 1e-5, 
                'w_phase1': 30,
                'w_phase2': 30,
                'w_phase3': 30,
                'gamma': GAMMA, 
                'explore': 1,
                }
    params = {"actPars": actPars, "valPars": valPars, "actTrain": actTrain, "valTrain": valTrain, "agents": agents}
    tanker = TRPOAgent(params, NAME, BoxTask("p_policy"))

if description == "CENTRAL_DDPG":
    agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot": {"sub": "/state", "pub": "/action"} #joint action space
            })

    valPars = {
                'neurons':      (15, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
                'trainMode':    True,
                'load':         False,
                'tau':          .005
                }        
    valTrain = {
                'batch':        128, 
                'lr':           3e-4, 
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'buffer':       10000,
                'gamma':        .99,
                'explore': (1, .3), #probability and sigma
                'baseExplore': .20,
                'decay': .90,
                'step': 200,
                'prioritySample': True,
                'a': 1,
                }
    actPars = {
                'neurons':      (13, 256, 256, 2),
                'act':          ['F.relu', 'F.relu'],
                'mean_range':   2,
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
            }
    actTrain = { 
                'lr':           1e-4, 
                }
    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, "agents": agents}
    tanker = Twin_DDPG(params, NAME, BoxTask("d_policy"))

if description == 'SAC':
    agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot":          {"sub": "/state", "pub": "/action"} #joint action space
            })

    valPars = {
                'neurons':      (13, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
                'trainMode':    True,
                'load':         False,
                'tau':          .005
                }        
    valTrain = {
                'batch':        128, 
                'lr':           3e-4, 
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'buffer':       10000,
                'explore':      False,
                'gamma':        .99
                }
    qPars = {
                'neurons':      (15, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0,
                                              3, 3]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1,
                                              3, 3]),
    }
    qTrain = {
                'lr':           1e-4, 
    }
    actPars = {
                'neurons':      (13, 256, 256, 2),
                'act':          ['F.relu', 'F.relu'],
                'mean_range':   2,
                'mu':           torch.Tensor([-.875, 0, .5, 0, 0, 0, 
                                              -.875, 0, .25,0, 0, 0, 0]),
                'std':          torch.Tensor([1.625, 1.25, .25, np.pi, np.pi, np.pi,
                                              1, 1, .25, np.pi, np.pi, np.pi, 1]),
            }
    actTrain = { 
                'lr':           1e-4, 
                'clamp':        (-20,.5), 
                'alpha':        .2, 
                }

    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, 'qPars': qPars, 'qTrain': qTrain, "agents": agents}
    tanker = SAC(params, NAME, BoxTask("p_policy"))

if description == 'FEUDAL':
    agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot": {"sub": "/state", "pub": "/action"} #joint action space
            })
    train = {
                'm_gamma':      .99,
                'w_gamma':      .8,
                'lr':           3e-4,
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'trainMode':    True,
                'clip_grad':    5,
                'step':         40,
                'alpha':        .1,
            }
    fun   = {
                's':            12,
                'u':            8,
                'c':            9,
                'k':            16,
                'd':            256,
            }
    params = {"agents": agents, 'train': train, 'fun': fun}
    agent = Feudal(params, NAME, BoxTask())

if description == 'HIRO':
    agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot":          {"sub": "/state", "pub": "/action"} #joint action space
            })
    valPars = {
                'neurons':      (14, 256, 256, 1),
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(14)]),
                'std':          torch.Tensor([1 for i in range(14)]),
                'trainMode':    True,
                'load':         False,
                'tau':          .005
                }        
    valTrain = {
                'batch':        128, 
                'lr':           3e-4, 
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'buffer':       10000,
                'explore':      False,
                'm_gamma':      .99,
                'w_gamma':      .8,
                'step':         32,
                }
    managerPars = {
                'neurons':      (12, 256, 256, 2),
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(12)]),
                'std':          torch.Tensor([1 for i in range(12)]),}
    managerTrain = {
                'lr':           3e-4,
                'c':            4}
    w_vPars = {
                'neurons':      (8, 256, 256, 8),
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(8)]),
                'std':          torch.Tensor([1 for i in range(8)]),               }
    w_vTrain = {
                    'lr':       3e-4}
    actPars = {
                'neurons':      (8, 256, 256, 8),
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(8)]),
                'std':          torch.Tensor([1 for i in range(8)]),
            }
    actTrain = { 
                'lr':           3e-4, }
    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, 
              "actTrain": actTrain, 'mPars': managerPars, 'mTrain': managerTrain, 
              "w_vPars": w_vPars, "w_vTrain": w_vTrain, "agents": agents}
    tanker = HIRO(params, NAME, BoxTask())
    

while(True):
    x = 1+1