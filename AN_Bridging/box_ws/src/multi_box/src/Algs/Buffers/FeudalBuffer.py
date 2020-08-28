import random
from collections import namedtuple
import numpy as np

# Taken from
# https://github.com/pytorch/tutorials/blob/master/Reinforcement%20(Q-)Learning%20with%20PyTorch.ipynb

Transition = namedtuple('Transition',
                        ('state','action', 'reward',
                         'mask', 'goal', 'policy', 'm_value', 'w_value_ext', 'w_value_int', 'm_state'))


class Memory(object):
    def __init__(self):
        self.memory = []
        self.position = 0

    def push(self, state, action, reward, mask, goal = None, 
            policy = None, m_value = None, w_value_ext = None, w_value_int = None, m_state = None):
        """Saves a transition."""
        self.memory.append(Transition(state, action, reward, mask,
                           goal, policy, m_value, w_value_ext, w_value_int, m_state))

    def sample(self, batch = 0):
        # Sample contiguous 
        if batch == 0:
            transitions = Transition(*zip(*self.memory))
            return transitions
        
        # Sample randoly
        c = np.random.choice(len(self.memory), batch)
        mem = map(self.memory.__getitem__, c)
        transitions = Transition(*zip(*mem))
        return transitions

    def __len__(self):
        return len(self.memory)