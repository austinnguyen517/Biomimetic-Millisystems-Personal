#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3 
from dstar_nav.msg import cliff


def proxyCliff(message):
    sendNewCliff.publish(message)

def proxyObstacle(message):
    sendNewObstacle.publish(message)

rospy.init_node("PlaceHolder", anonymous = True)
sendNewObstacle = rospy.Publisher("/proxyObstacle", Vector3, queue_size = 100)
sendNewCliff = rospy.Publisher("/proxyCliff", cliff, queue_size = 100)
receiveNewCliff = rospy.Subscriber("/newCliff", cliff, proxyCliff, queue_size = 100)
receiveNewObstacle = rospy.Subscriber("/newObstacle", Vector3, proxyObstacle, queue_size = 100)

while (True):
    #do something random
    x = 1+1