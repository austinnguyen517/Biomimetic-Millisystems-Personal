#! /usr/bin/env python

import numpy as np 
import torch 
import torch.nn as nn
import math
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs. msg import Vector3
from collections import OrderedDict

from Algs.FuN import Feudal 
from Algs.Counterfactual import Counter
from Algs.CounterFeudal import CounterFeudal
from Algs.ContinuousCounterFactual import CounterContinuous
from Tasks.boxDoubleTask import BoxDoubleTask

NAME = 'a_bot'
NAMETWO = 'b_bot'

algs = {
    10: "COUNTER",
    11: "COUNTER_FEUDAL",
    12: "COUNTER_CONT"
}
ALGORITHM = 10
description = algs[ALGORITHM]
rospy.init_node('Dummy', anonymous = True)

if description == "COUNTER":
    agents = OrderedDict({
                #ensure ordering matches ros message
                "a_bot": {"sub": "/state", "pub": "/action1"}, #joint action space
            })
    agents["b_bot"] = {"sub": "/state", "pub": "/action2"}

    actPars = {
                #define hidden state size and input state size...
                'h_state_n':    256,
                'x_state_n':    11, #4 robot, 4 box, 3 observation
                'u_n':          6,
                'mu':           torch.Tensor([0 for i in range(11)]),
                'std':          torch.Tensor([1 for i in range(11)]),
                'share_params': True
            }
    actTrain = { 
                'lr':           5e-4, # this is what they used in the paper...but they used RMSProp not Adam
                'clip':         1
                }

    valPars = {
                'neurons':      (14, 256, 256, actPars['u_n']), #true state: 12, actions other: 1, ID
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(14)]),
                'std':          torch.Tensor([1 for i in range(14)]),
                'trainMode':    True,
                'load':         False,
                }        
    valTrain = {
                'lr':           3e-4, 
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'buffer':       5000,
                'gamma':        .99,
                'batch':        32,
                }
    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, "agents": agents}
    tanker = Counter(params, NAME, BoxDoubleTask())

if description == "COUNTER_FEUDAL":

    agents = OrderedDict({
                #ensure ordering matches ros message
                "a_bot": {"sub": "/state", "pub": "/action1"}, #joint action space
            })
    agents["b_bot"] = {"sub": "/state", "pub": "/action2"}

    mPars = {
                'neurons':      (15, 256, 256, 2),
                'act':          ['F.leaky_relu', 'F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(15)]),
                'std':          torch.Tensor([1 for i in range(15)]),
                'c':            2,
    }
    mTrain = {
                'lr':           3e-4,
                'gamma':        .99
            }
    
    actPars = {
                'neurons':      (8, 256, 256, 2), # 6 state, 3 others, 2 goal
                'act':          ['F.leaky_relu', 'F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(8)]),
                'std':          torch.Tensor([1 for i in range(8)]),
            }
    actTrain = { 
                'lr':           3e-4, 
                }

    valPars = { # counterfactual network
                'neurons':      (22, 256, 256, 1), # Input: true_state = 18, actions = 2*2  Output: 1
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(22)]),
                'std':          torch.Tensor([1 for i in range(22)]),
                'trainMode':    True,
                'load':         False,
                'tau':          .005,
                }        
    valTrain = {
                'lr':           3e-4, 
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'buffer':       10000,
                'gamma':        .99,
                'batch':        5,
                }
    
    localPars = { # counterfactual network
                'neurons':      (8, 256, 256, 1), # Input: state = 6, Others: 3, 2 goal, Output: 1
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(8)]),
                'std':          torch.Tensor([1 for i in range(8)]),
                }   
    localTrain = {
                'lr':           3e-4
                }
    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, 
            "m_pars": mPars, "m_train": mTrain, 'local_pars': localPars, 'local_train': localTrain,"agents": agents}
    tanker = CounterFeudal(params, NAME, BoxDoubleTask())

if description == "COUNTER_CONT":
    agents = OrderedDict({
                #ensure ordering matches ros message
                "a_bot": {"sub": "/state", "pub": "/action1"}, #joint action space
            })
    agents["b_bot"] = {"sub": "/state", "pub": "/action2"}


    actPars = {
                'neurons':      (15, 256, 256, 2),
                'act':          ['F.leaky_relu', 'F.leaky_relu'],
                #define hidden state size and input state size...
                'h_state_n':    128,
                'x_state_n':    15, #6 robot, 6 box, 3 observation
                'u_n':          2,
                'mu':           torch.Tensor([0 for i in range(15)]),
                'std':          torch.Tensor([1 for i in range(15)]),
                'share_params': True,
                'mean_range':   3
            }
    actTrain = { 
                'lr':           3e-4, 
                'clip':         1,
                'clamp':        [0, 1, -2.5], # starting lower bound, upper bound, ending lower bound
                }

    valPars = {
                'neurons':      (22, 256, 256, 1), #true state: 18, actions: 4
                'act':          ['F.leaky_relu','F.leaky_relu'],
                'mu':           torch.Tensor([0 for i in range(22)]),
                'std':          torch.Tensor([1 for i in range(22)]),
                'trainMode':    True,
                'load':         False,
                }        
    valTrain = {
                'lr':           3e-4, 
                'w_phase1':     1,
                'w_phase2':     1, 
                'w_phase3':     1,
                'buffer':       10000,
                'gamma':        .99,
                'batch_size':   2,
                }
    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, "agents": agents}
    tanker = CounterContinuous(params, NAME, BoxDoubleTask())