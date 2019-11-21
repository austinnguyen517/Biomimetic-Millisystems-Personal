# Biomimetic-Millisystems
Path planning, perception, and collaborative robot learning

## AN_PathPlanning
#### dStar_ws
  - Catkin workspace using ROS interface for collaborative robots
  - Uses multiple robots in V-Rep environment to navigate unpredicatable terrain using revised D-StarLite algorithm
  - Utilizes proxy ROS node to effectively communicate new obstacles between robots
#### misc
  - Updated versions of DStar agent using V-Rep API
  
## Miscellaneous Files
#### DStarAgent, Environment, Legged
  - Older version using V-Rep API (as opposed to ROS) for single robot DStarLite navigation
#### SimpleAgent
  - ColorFollower: Uses V-Rep API and greedily searches for highest color intensity by extracting information from RGB arrays
#### Various V-Rep Scenes
