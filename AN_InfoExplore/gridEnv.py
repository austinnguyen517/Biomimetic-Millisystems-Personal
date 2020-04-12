#Environment defined here
#Each robot has a local map with no global map. Almost decentralized. 
#Only interact with other local maps if within a certain radius of another robot. Trade information as such through a proxy?? Or just directly

#Define a different class that inherits from this class. Call it parentGridWorld or something to represent the global map shared by all the robots
#This class is for the LOCAL maps

class gridWorld():
    def __init__(self, size, resolution, threshold = .95):
        self.size = size #in meters 
        self.resolution = resolution  #cells per meter
        self.mapDim = (size[0] * resolution, size[1] * resolution)
        self.map = (np.empty([self.mapDim[0], self.mapDim[1], 2])).fill(.5)
        self.thresh = threshold

    def neighbors(self, x, y):
        result = []
        for i in [-1,0,1]:
            for j in [-1, 0, 1]:
                if !(i == 0 and j==0) and self.map[i,j,0] < self.thresh:
                    result.append((x+i, x+j))
        return result
    
    def entropy(self):
        total = 0
        for i in range(self.mapDim[0]):
            for j in range(self.mapDim[1]):
                total += (-1) * self.map[i,j,0] * np.log(self.map[i,j,0])
        return total
    
    def getUtility(self, x, y):
        return self.map[x,y,1]
    
    def setUtility(self, x, y, value):
        self.map[x,y,1] = value 
    
    def getProb(self, x, y):
        return self.map[x,y,0]
    
    def setProb(self, x, y, value):
        if value >= self.thresh:
            value = 1
        if value <= (1-self.thresh):
            value = 0
        self.map[x,y,0] = value

    def euclidian(self, point1, point2):
        return np.sqrt(np.pow(point1[0] - point2[0], 2) + np.pow(point1[1] - point2[1], 2))
    
    def edge(self, point1, point2):
        return self.euclidian(point1, point2)