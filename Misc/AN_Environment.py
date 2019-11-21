'''Class for map environment. Easier for keeping track of variables and functionality.'''

'''Not used just yet...but should use soon to be more organized'''
import numpy as np
import matplotlib.pyplot as plt
import sys
import heapq
import math
import operator


class Environment:

        def __init__(self, res, mapDimensions, cliffThreshold):
            self.robotPosition = None
            self.goalPosition = None
            self.mapDimensions = mapDimensions
            self.res = res #how many nodes per meter we want to process. Increase resolution to take into account more nodes...but might slow down processing
            self.map = None
            self.obstacles = set()
            self.cliffs = {}
            self.slopes = set()
            self.cliffThreshold = cliffThreshold
            '''TODO: check these are right after transposing our graph'''
            self.matrices = {(-1,1): -1 * diagonalLeft,
                            (0,1): -1 * vertical,
                            (1,1): diagonalRight,
                            (-1,0): -1 * horizontal,
                            (1,0): horizontal,
                            (-1,-1): -1 * diagonalRight,
                            (0,-1): vertical,
                            (1,-1): diagonalLeft} #returns appropriate matrix depending on vector

        def updateRobotPosition(self, point):
            self.robotPosition = point

        def updateGoal(self, point):
            self.goalPosition = point

        def getRobotPosition(self):
            return self.robotPosition

        def getGoal(self):
            return self.goalPosition

        def getMapDimensions(self):
            return self.mapDimensions

        def getMap(self, x, y, z):
            return self.map[x,y,z]

        def setMap(self, x, y, z, val):
            self.map[x,y,z] = val

        def initializeMap(self):
            ############### NAIVELY FILL IN G AND RHS VALUES FOR MAP #########################
            self.map = np.zeros(self.mapDimensions)
            for i in range(self.mapDimensions[0]):
                for j in range(self.mapDimensions[1]):
                    cost = self.euclidian(self.goalPosition, (i,j))
                    self.map[i,j,0] = cost
                    self.map[i,j,1] = cost
                    self.map[i,j,2] = 0
                    self.map[i,j,3] = 0

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
            return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1])**2) **(1/2)

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
            high1 = point1 in self.cliffs.keys()
            low2 = tuple(map(operator.sub, point2, point1)) in self.cliffs[point1] if high1 else False
            if (high1 and low2): #if there is a significant height difference between the two points
                return np.inf
            high2 = point2 in self.cliffs.keys()
            d = self.euclidian(point1, point2)
            if high2:
                print("WARNING: Next step has cliff hazard!")
                return d + 10
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

        def updateSlope(self, x, y, slope):
            filter = np.zeros(FILTERSHAPE) + slope
            left = x - FILTERSHAPE[0] // 2
            bot = y - FILTERSHAPE[1] // 2
            for i in range(FILTERSHAPE[0]):
                for j in range(FILTERSHAPE[1]):
                    self.setMap(left + i, bot + j, 2, .5 * filter[i,j] + .5 * self.getMap(i,j,2))#filter[i,j])

        def updateHeight(self, x, y, vector, h = None):
            if any(vector) and not h:
                filter = self.filter(x,y, vector)
                alpha = .5
            else:
                filter = (np.zeros(FILTERSHAPE) + 1) * h
                alpha = 1
            left = x - FILTERSHAPE[0] // 2
            bot = y - FILTERSHAPE[1] // 2
            for i in range(FILTERSHAPE[0]):
                for j in range(FILTERSHAPE[1]):
                    self.setMap(left + i, bot + j, 3, alpha * filter[i,j] + (1-alpha) * self.getMap(i,j,3))#filter[i,j])

        def filter(self, x, y, vector):
            #Returns a matrix that represents elevations based on slopes.
            slope = self.getMap(x,y,2)
            xyVec = (vector[0], vector[1])
            #this is the normal vector of the surface. This will correspond to the direction from (x,y) to height we should base current elevation on
            adjacents = self.neighbors((0,0), condition = True)
            dots = []
            for v in adjacents:
                val = self.dotProduct(v, xyVec)
                dots += [val]
            index = dots.index(max(dots))
            ref = adjacents[index]
            deltaH = self.euclidian(self.inverseTransform(ref), self.inverseTransform((x,y))) * slope
            newHeight = self.getMap(ref[0], ref[1], 3) + deltaH

            #Compute the matrix based on the direction of our vector
            ref = (-ref[0], -ref[1]) #find the direction our slope is actually pointing (not the normal vector)
            matrix = self.matrices[ref]
            filter = newHeight * (np.zeros(FILTERSHAPE) + 1) + slope * matrix
            filter = np.clip(filter, a_min = 0, a_max = float('inf'))
            return filter



FILTERSHAPE = (5,5)

diagonalRight = np.array([[-4, -3, -2, -1, 0],
                          [-3, -2, -1, 0, 1],
                          [-2, -1, 0, 1, 2],
                          [-1, 0, 1, 2, 3],
                          [0, 1, 2, 3, 4]])
diagonalLeft = np.array([[0, 1, 2, 3, 4],
                         [-1, 0, 1, 2, 3],
                         [-2, -1, 0, 1, 2],
                         [-3, -2, -1, 0, 1],
                         [-4, -3, -2, -1, 0]])
vertical = np.array([[2, 2, 2, 2, 2],
                    [1, 1, 1, 1, 1],
                    [0, 0, 0, 0, 0],
                    [-1, -1, -1, -1, -1],
                    [-2, -2, -2, -2, -2]])
horizontal = np.array([[-2, -1, 0, 1, 2],
                      [-2, -1, 0, 1, 2],
                      [-2, -1, 0, 1, 2],
                      [-2, -1, 0, 1, 2],
                      [-2, -1, 0, 1, 2]])
