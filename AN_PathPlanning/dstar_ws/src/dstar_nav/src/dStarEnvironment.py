'''Class for map environment. Easier for keeping track of variables and functionality.'''

'''Not used just yet...but should use soon to be more organized'''
import numpy as np
import matplotlib.pyplot as plt
import sys
import heapq
import math
import operator

EDGEBUFF = 20


class Environment:

        def __init__(self, res, mapDimensions, cliffThreshold):
            self.robotPosition = None
            self.robotPositionUT = None
            self.goalPosition = None
            self.mapDimensions = mapDimensions
            self.res = res #how many nodes per meter we want to process. Increase resolution to take into account more nodes...but might slow down processing
            self.map = None
            self.obstacles = set()
            self.cliffs = {}
            self.slopes = set()
            self.cliffThreshold = cliffThreshold
            
        def initializeMap(self):
            ############### NAIVELY FILL IN G AND RHS VALUES FOR MAP #########################
            self.map = np.zeros(self.mapDimensions)
            for i in range(self.mapDimensions[0]):
                for j in range(self.mapDimensions[1]):
                    cost = self.euclidian(self.goalPosition, (i,j))
                    self.map[i,j,0] = cost
                    self.map[i,j,1] = cost

        def dotProduct(self, v1, v2):
            #given a point, computes the absolute value of dot product of vector: start to point and vector: start to goal
            x1 = v1[0]
            y1 = v1[1]
            norm1 = (x1**2 + y1**2)**(1/2)
            x2 = v2[0]
            y2 = v2[1]
            norm2 = (x2**2 + y2**2)**(1/2)
            #must include normalizations
            return (x1/norm1) * (x2/norm2) + (y1/norm1) * (y2/norm2)

        def euclidian(self, point1, point2):
            return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1])**2)

        def neighbors(self, point, condition = False):
            #returns all the neighbors in array coordinates of a given point
            #condition toggles whether we allow all 9 points
            neighbors = []
            for i in [-1,0,1]:
                for j in [-1,0,1]:
                    curr = (point[0] + i, point[1] + j)
                    if (condition or (curr[0] >= 0 and curr[1] >= 0 and curr[0] < self.mapDimensions[0] and curr[1]<self.mapDimensions[1])) and curr != point:
                        neighbors += [curr]
            return neighbors

        def transform(self, point):
            #find the place in the array a point (in v-rep coordinates) is
            x = int(point[0] * self.res)
            y = int(point[1] * self.res)
            return (x,y)

        def inverseTransform(self, point):
            x = point[0] / self.res
            y = point[1] / self.res
            return (x,y)

        def rotate(self, unitVector, angles): #transforms a vector put in the coordinate frame of our sensor into the world coordinate frame. angles represents the current orientation of our ROBOT
            #for this instance, we must rotate around X by pi/2 radians (z axis of sensor is facing forward)
            #Then, rotate about z axis by math.pi/2 + angles[2] radians (accounts for the fact that the sensor is oriented a certain way)
            unitVector = np.array(unitVector)
            unitVector = np.dot(self.rotationX(math.pi/2 - angles[0]), unitVector)
            unitVector = np.dot(self.rotationZ(math.pi/2 + angles[2]), unitVector)
            return unitVector

        def rotationX(self, theta):
            return np.array([[1,0,0], [0, math.cos(theta), -math.sin(theta)], [0, math.sin(theta), math.cos(theta)]])

        def rotationZ(self, theta):
            return np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0,0,1]])

        def edge(self, point1, point2): #abstraction for edges
            #high1 = point1 in self.cliffs.keys()
            #low2 = tuple(map(operator.sub, point2, point1)) in self.cliffs[point1] if high1 else False
            #if (high1 and low2): #if there is a significant height difference between the two points
            #    return np.inf
            #high2 = point2 in self.cliffs.keys()
            d = self.euclidian(point1, point2)
            nearObstacle = any([self.map[n[0], n[1], 0] == np.inf for n in self.neighbors(point2)])
            #if high2 or nearObstacle:
            if nearObstacle:
                return d + EDGEBUFF
            return d

        def radToDeg(self, rad):
            #points are returned in radians by orientation
            return (rad/math.pi) * 180

        def goalDone(self):
            return self.robotPosition == self.goalPosition

        def showColorGrid(self):
            plt.title('G values: Black is Current Position')
            temp = self.map[self.robotPosition[0],self.robotPosition[1], 0]
            self.map[self.robotPosition[0], self.robotPosition[1], 0] = 0
            heatmap = plt.pcolor(self.map[:,:,0].T)
            self.map[self.robotPosition[0], self.robotPosition[1], 0] = temp
            plt.show()
            '''plt.title('Slopes')
            another = plt.pcolor(self.map[:,:,2].T)
            plt.show()
            plt.title('Elevation')
            another = plt.pcolor(self.map[:,:,3].T)
            plt.show()'''

        def analyzeCliffs(self, matrix):
            #given a 2D matrix of heights
            shape = matrix.shape
            midVal = matrix[shape[0]//2, shape[1]//2]
            norm = np.around((matrix - midVal), decimals = 1) #take absolute value, rounds values to integer
            result = set()
            xSpace = shape[0] // 3
            ySpace = shape[1] // 3
            if np.mean(norm[0:xSpace, 0:ySpace]) > self.cliffThreshold: result.add((-1,1))
            if np.mean(norm[xSpace: 2 * xSpace, 0:ySpace]) > self.cliffThreshold: result.add((-1,0))
            if np.mean(norm[2*xSpace:, 0:ySpace]) > self.cliffThreshold: result.add((-1,-1))
            if np.mean(norm[0:xSpace, ySpace: 2*ySpace]) > self.cliffThreshold: result.add((0,1))
            if np.mean(norm[2*xSpace:, ySpace: 2*ySpace]) > self.cliffThreshold: result.add((0,-1))
            if np.mean(norm[0:xSpace, 2*ySpace:]) > self.cliffThreshold: result.add((1,1))
            if np.mean(norm[xSpace: 2 * xSpace, 2*ySpace:]) > self.cliffThreshold: result.add((1,0))
            if np.mean(norm[2*xSpace:, 2*ySpace:]) > self.cliffThreshold: result.add((1,-1))
            return result
        
        def tuplesToList(self, input):
            #converts a list of tuples to a normal list
            res = []
            for tup in input:
                res.append(tup[0])
                res.append(tup[1])
            return res
        
        def listToTuples(self, input):
            #converts a list to tuples
            assert len(input) % 2 == 0
            res = []
            for i in range(0, len(input) / 2, 2):
                res.append((input[i], input[i+1]))
            return res
