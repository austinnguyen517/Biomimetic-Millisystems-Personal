#! /usr/bin/env python

import numpy as np 
import torch 
import torch.nn as nn
import math 
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs. msg import Vector3
from collections import OrderedDict

from Algs.doubleQ import DoubleQ
from Algs.TD3 import Twin_DDPG
from Algs.A2C import A2C
from Algs.SoftActorCritic import SAC
from Tasks.moveTask import MoveTask

NAME = 'bot'

algs = {
    3: "DOUBLE_Q", #REMINDER: if choosing 3, make sure to only run the tankAgent.py in the launch file
    5: "TWIN_DDPG",
    6: "A2C",
    7: "SAC"
}
ALGORITHM = 7
description = algs[ALGORITHM]
rospy.init_node('Dummy', anonymous = True)

if description == "DOUBLE_Q":
    agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot":          {"sub": "/state", "pub": "/action"}
            })
    valPars = {
                'neurons':      (2, 256, 256, 7),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([0, 0]),
                'std':          torch.Tensor([2, 2]),
                'trainMode':    True,
                'load':         False, 
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
                'explore': .4, 
                'baseExplore': .1,
                'decay': .6,
                'step': 50,
                'double': True,
                'prioritySample': True,
                'a': 1
                }
    params = {"valPars": valPars, "valTrain": valTrain, "agents": agents}
    tanker = DoubleQ(params, NAME, MoveTask("argmax"))

if description == "TWIN_DDPG":
   agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot": {"sub": "/state", "pub": "/action"} #joint action space
            })

    valPars = {
                'neurons':      (4, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([0, 0, 0, 0]),
                'std':          torch.Tensor([2, 2, 3, 3]),
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
                'gamma':        .99
                'explore': (1, .3), #probability and sigma
                'baseExplore': .20,
                'decay': .90,
                'step': 200,
                'prioritySample': True,
                'a': 1,
                }
    actPars = {
                'neurons':      (2, 256, 256, 2),
                'act':          ['F.relu', 'F.relu'],
                'mean_range':   2,
                'mu':           torch.Tensor([0, 0]),
                'std':          torch.Tensor([2, 2]),
            }
    actTrain = { 
                'lr':           1e-4, 
                }
    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, "agents": agents}
    tanker = Twin_DDPG(params, NAME, MoveTask("d_policy"))

if description == "A2C":
   agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot":   {"sub": "/state", "pub": "/action"} #joint action space
            })

    valPars = {
                'neurons':      (2, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([0, 0]),
                'std':          torch.Tensor([3, 3]),
                'trainMode':    True,
                'load':         False,
                'tau':          .005
                }        
    valTrain = {
                'batch':        256, 
                'lr':           3e-4, 
                'buffer':       10000,
                'nu':           .999, 
                'explore':      False,
                'gamma':        .99
                }
    actPars = {
                'neurons':      (2, 256, 256, 2),
                'act':          ['F.relu', 'F.relu'],
                'mean_range':   3,
                'mu':           torch.Tensor([0, 0]),
                'std':          torch.Tensor([3, 3])
            }
    actTrain = { 
                'lr':           1e-4, 
                }
    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, "agents": agents}
    tanker = A2C(params, NAME, MoveTask("p_policy"))

if description == "SAC":
    agents = OrderedDict({
                #ensure ordering matches ros messages
                "bot":          {"n": 2, "u": 2 ,"sub": "/state", "pub": "/action"} #joint action space
            })

    valPars = {
                'neurons':      (2, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([0, 0]),
                'std':          torch.Tensor([3, 3]),
                'trainMode':    True,
                'load':         False,
                'tau':          .005
                }        
    valTrain = {
                'batch':        256, 
                'lr':           3e-4, 
                'buffer':       10000,
                'nu':           .999, 
                'explore':      False,
                'gamma':        .99
                }
    qPars = {
                'neurons':      (4, 256, 256, 1),
                'act':          ['F.relu','F.relu'],
                'mu':           torch.Tensor([0, 0, 0, 0]),
                'std':          torch.Tensor([4, 4, 3, 3]),
    }
    qTrain = {
                'lr':           1e-4, 
    }
    actPars = {
                'neurons':      (2, 256, 256, 2),
                'act':          ['F.relu', 'F.relu'],
                'mean_range':   3,
                'mu':           torch.Tensor([0, 0]),
                'std':          torch.Tensor([3, 3])
            }
    actTrain = { 
                'lr':           1e-4, 
                'clamp':        (-20,.5), 
                'alpha':        .2, 
                }

    params = {"valPars": valPars, "valTrain": valTrain, "actPars": actPars, "actTrain": actTrain, 'qPars': qPars, 'qTrain': qTrain, "agents": agents}
    tanker = SAC(params, NAME, MoveTask("p_policy"))


while(True):
    x = 1+1