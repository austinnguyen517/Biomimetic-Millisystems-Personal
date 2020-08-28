import random
from collections import namedtuple
import numpy as np

# Taken from
# https://github.com/pytorch/tutorials/blob/master/Reinforcement%20(Q-)Learning%20with%20PyTorch.ipynb

Transition = namedtuple('Transition',('state','action', 'reward','mask', 'next_action', 'next_state', 'local', 'next_local', 'policies'))

class Memory(object):
    def __init__(self, size = 10000):
        self.memory = []
        self.position = 0
        self.size = size

    def push(self, state, action, reward, mask, next_action, next_state, local=None, next_local=None, policies=None):
        """Saves a transition."""
        if len(self.memory) >= self.size:
            self.memory.pop(0)
        self.memory.append(Transition(state, action, reward, mask, next_action, next_state, local, next_local, policies))

    def sample(self, batch = 0):
        # Sample contiguous 
        if batch == 0:
            transitions = Transition(*zip(*self.memory))
            return transitions
        
        # Sample randoly
        #probs = np.arange(1, len(self.memory) + 1)
        #p = np.true_divide(probs, probs.sum())
        c = np.random.choice(len(self.memory), batch)
        return self.get_transitions(c)
    
    def get_transitions(self, choices):
        mem = map(self.memory.__getitem__, choices)
        transitions = Transition(*zip(*mem))
        return transitions

    def __len__(self):
        return len(self.memory)
