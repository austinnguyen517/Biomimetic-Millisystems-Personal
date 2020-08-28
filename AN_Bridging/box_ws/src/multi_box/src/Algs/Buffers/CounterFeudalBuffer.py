import random
from collections import namedtuple
import numpy as np

# Taken from
# https://github.com/pytorch/tutorials/blob/master/Reinforcement%20(Q-)Learning%20with%20PyTorch.ipynb

# Store: true state, local states, local actions, local policies, goals, goal log_prob, goal means, reward, done, 
# next actions, next true states, next local states
batched_transition = namedtuple('Transition', ('states', 'actions', 'rewards', 'masks', 'next_state', 'local','goals'))

class Memory(object):
    def __init__(self):
        self.memory = []
        self.transition = batched_transition

    def push(self, batch):
        """Saves a transition."""
        self.memory.append(batch)

    def sample(self, batch = 0):
        # Sample contiguous
        if batch == 0:
            transitions = self.transition(*zip(*self.memory))
            return transitions

        # Sample randomly
        c = np.random.choice(len(self.memory), batch, replace = False)
        mem = map(self.memory.__getitem__, c)
        # Remove from list due to updated goals
        for i in sorted(list(c), reverse = True):
           self.memory.pop(i)

        return mem

    def __len__(self):
        return len(self.memory)
    