# Biomimetic-Millisystems: Overview
  #### Focus: 
  - Multi-agent reinforcement learning approaches for collaborative locomotion
  - Implementations of centralized value-based approaches, distributed actor critic methods and hierarchical decomposition algorithms
  - Geared towards increasing scalability for multi-agent learning and applications for locomotion through difficult terrain
  #### Other Misc:
  - Path planning with revised DStar Lite algorithm for collaborative robot navigation
  - Perception using RGB values and greedy algorithms
  - ROS interface to design of ROS infrastructure to facilitate multirobot updates
  
# AN_Bridging
## ROS Workspaces:
  - Box_ws: Collaborative Box-Pushing for Bridging 
  - Bridge_ws: Tether-based Approach for Bridging
## Implemented Algorithms:
  - Double DQN with dual networks and TD Error Priority Sampling
  - Q-SARSA
  - TD3 (Twin Delayed DDPG)
  - Soft Actor Critic ~ coming soon
  - MADDPG ~ coming soon
  - Hierarchical Feudal-Inspired Algorith ~ coming soon

# AN_Info_Explore
## Information-based exploration for 3D mapping
  - Unfinished

# AN_PathPlanning
#### dStar_ws
  - Catkin workspace using ROS interface for collaborative robots
  - Uses multiple robots in V-Rep environment to navigate unpredicatable terrain using revised D-StarLite algorithm
  - Utilizes proxy ROS node to effectively communicate new obstacles between robots
 
# AN_RGB_Perception
#### Red light follower
  - Naive, greedy implementation for robot following red light
