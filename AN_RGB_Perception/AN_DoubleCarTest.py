import vrep
import numpy as np
import matplotlib as plt
import sys

vrep.simxFinish(-1) #clean up the previous stuff
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection unsuccessful')
    sys.exit('Error: Could not connect to API server')

frontMotorHandles = [-1,-1,-1,-1]
error, frontMotorHandles[0] = vrep.simxGetObjectHandle(clientID, 'joint_front_left_wheel#0', vrep.simx_opmode_oneshot_wait)
error, frontMotorHandles[1] = vrep.simxGetObjectHandle(clientID, 'joint_front_right_wheel#0', vrep.simx_opmode_oneshot_wait)
error, frontMotorHandles[2] = vrep.simxGetObjectHandle(clientID, 'joint_back_right_wheel#0', vrep.simx_opmode_oneshot_wait)
error, frontMotorHandles[3] = vrep.simxGetObjectHandle(clientID, 'joint_back_left_wheel#0', vrep.simx_opmode_oneshot_wait)

vrep.simxSetJointTargetVelocity(clientID, frontMotorHandles[0], 3, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, frontMotorHandles[1], -2, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, frontMotorHandles[2], -2, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, frontMotorHandles[3], 3, vrep.simx_opmode_streaming)
