'''

Simple agent. Basically follows red light inside of the red cube. Is able to find the glowing cube
even when not initially fully visible (ie behind a block). Just follows the place that has max redness.

Still needs to account for running into walls (proximity sensors) and not getting too confused with deciding
which path to take.'''


import vrep
import numpy as np
import matplotlib.pyplot as plt
import sys

LSignalName = "CycleLeft"
RSignalName = "CycleRight"
REDBENCH = 240
FINISH = .8
CENTER = .15

class SimpleAgent:
    def __init__(self, client, leftName, rightName):
        self.clientID = client
        self.LSignalName = leftName
        self.RSignalName = rightName
        self.CycleFreqL = 1
        self.CycleFreqR = 1
        self.camHandles = [None, None, None] #left, front, right
        self.numCameras = len(self.camHandles)
        self.proxHandle = None
        self.dangerWall = False
        self.resolution = None
        self.image = None
        self.pureRed = None #this is a 2D array that specifies that significance of a pixel. Sums reds subtracts greens and blues...looking for most pure red thing
        self.reds = None
        self.hasImage = False
        self.prevMode = None
        self.prevTup = None
        self.duration = 120000

    def policy(self):
        #simple policy. If there is the red light in front, keep going forward. Otherwise, look around until there is one
        mode = "circle" #three modes: circle, forward, and center. Start with circle to search for light
        while (not self.hasImage):
            self.getImage()
        while(not self.goalDone()): #if there is an image and the goal is done, exit
            print(mode)
            mode = self.circle(mode)
            mode = self.forward(mode)
            mode = self.center(mode)
            self.getImage()
            mode = self.avoidWall(mode)
            #self.displayImage()
        print("GOAL ACHIEVED")
        self.clearSignal()

    def circle(self, mode):
        #look around until you find red in the image. Returns a different mode if needed.
        if mode == "circle":
            if self.hasRed(): #if there is a red pixel, center it
                return "center"
            self.CycleFreqL = 4
            self.CycleFreqR = 1
            self.sendSignal()
        return mode

    def forward(self, mode):
        #go forward as long as there is red in the middle of the image. Returns a different node if needed
        if mode == "forward":
            if not self.hasRed():
                return "circle"
            redCentered, redLoc = self.redCenter()
            if not redCentered:
                return "center"
            self.CycleFreqL = 4
            self.CycleFreqR = 4
            self.sendSignal()
        return mode

    def center(self, mode):
        #center yourself so that red is in the middle of the image. Returns a different mode if needed
        if mode == "center":
            if not self.hasRed():
                return "circle"
            redCentered, redLoc = self.redCenter()
            if redCentered:
                return "forward"
            offset = redLoc - (self.resolution[1] / 2) #offset equals a positive number if red is too far to the right. Negative if too far to left
            print(offset)
            if offset >= 0:
                self.CycleFreqL = 4
                self.CycleFreqR = 2
            else:
                self.CycleFreqL = 2
                self.CycleFreqR = 4
            self.sendSignal()
        return mode

    def clearSignal(self):
        vrep.simxClearFloatSignal(self.clientID, self.LSignalName, vrep.simx_opmode_oneshot)
        vrep.simxClearFloatSignal(self.clientID, self.RSignalName, vrep.simx_opmode_oneshot)

    def hasRed(self):
        #returns true if an image has ANY red in it
        value = (self.reds > REDBENCH).sum()
        return value > 0

    def redCenter(self):
        #returns true and 0 if the image has red in the center square. Otherwise, it reutnrs false and the x location of the red relative to center
        indexMax = np.argmax(self.pureRed)
        x = indexMax % self.resolution[1]
        lowerBound = (.5 - (CENTER / 2)) * self.resolution[1]
        upperBound = self.resolution[1] - lowerBound
        if x < upperBound and x > lowerBound:
            return True, 0
        else:
            return False, x

    def goalDone(self):
        #basically, if the red part of RGB is covered 80% in significant red values, then we can safely say that we are at the red cube
        cutoff = int(self.resolution[1]/self.numCameras)
        frontPixels = self.reds[:, cutoff : 2*cutoff]
        numPixelsRed = (frontPixels > REDBENCH).sum()
        finished = frontPixels.size * FINISH
        return numPixelsRed >= finished

    def checkProximity(self):
        error, state, point, handle, vector= vrep.simxReadProximitySensor(self.clientID, self.proxHandle,vrep.simx_opmode_buffer)
        self.dangerWall = (state, point)

    def avoidWall(self, mode):
        #we need to check this regardless of what mode we are currently in
        self.checkProximity()
        if self.dangerWall[0]: #if there is a wall
            prevTup = (self.CycleFreqL, self.CycleFreqR)
            for i in range(self.duration):
                self.CycleFreqL = - prevTup[1]
                self.CycleFreqR = - prevTup[0]
                self.sendSignal()
        return mode

    def prepareVision(self):
        rCode1, self.camHandles[1] = vrep.simxGetObjectHandle(self.clientID,'FrontCamera',vrep.simx_opmode_oneshot_wait)
        rCode2, self.camHandles[2] = vrep.simxGetObjectHandle(self.clientID,'RightCamera',vrep.simx_opmode_oneshot_wait)
        rCode3, self.camHandles[0] = vrep.simxGetObjectHandle(self.clientID,'LeftCamera',vrep.simx_opmode_oneshot_wait)
        if rCode1 != vrep.simx_return_ok or rCode2 != vrep.simx_return_ok or rCode3 != vrep.simx_return_ok:
            print("Could not get vision sensor handle. Exiting.")
            sys.exit()
        for handle in self.camHandles:
            error, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, handle, 0, vrep.simx_opmode_streaming)
        self.resolution = resolution
        self.image = image

    def prepareProximity(self):
        rCode, self.proxHandle = vrep.simxGetObjectHandle(self.clientID, 'FrontProxSensor', vrep.simx_opmode_oneshot_wait)
        if rCode != vrep.simx_return_ok:
            print("Could not get proximity sensor handle. Exiting.")
            sys.exit()
        error, state, point, handle, vector = vrep.simxReadProximitySensor(self.clientID, self.proxHandle, vrep.simx_opmode_streaming)
        print(state)
        print(point)
        self.checkProximity()

    def sendSignal(self):
        vrep.simxSetFloatSignal(self.clientID, self.LSignalName, self.CycleFreqL, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(self.clientID, self.RSignalName, self.CycleFreqR, vrep.simx_opmode_oneshot)

    def getImage(self):
        images = []
        for handle in self.camHandles:
            error, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, handle, 0, vrep.simx_opmode_buffer)
            if error != vrep.simx_return_ok:
                return 0
            else:
                image = np.array(image, dtype = np.uint8)
                image.resize(resolution[0], resolution[1], 3) #rgb valuess
                images += [image]
        self.image = np.concatenate(images, axis = 1) #basically make a giant screen of photos
        self.resolution = [resolution[0], resolution[1] * self.numCameras]
        self.processImage()
        self.hasImage = True

    def processImage(self):
        #given an image in self.image, translate it from a 3D array (RGB) into a 2D array with values that correspond to how purely red it is
        reds = self.image[:,:,0]
        greens = self.image[:,:,1]
        blues = self.image[:,:,2]
        self.pureRed = reds - greens - blues
        self.reds = reds

    def displayImage(self):
        if self.hasImage:
            plt.imshow(self.image)
            plt.show()
        return 0

vrep.simxFinish(-1) #clean up the previous stuff
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection unsuccessful')
    sys.exit('Error: Could not connect to API server')

agent = SimpleAgent(clientID, LSignalName, RSignalName)
agent.clearSignal()
agent.prepareVision()
agent.prepareProximity()
agent.policy()
