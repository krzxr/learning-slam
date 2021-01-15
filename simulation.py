import numpy as np 
import matplotlib.pyplot as plt
def pathGeneration(walls):
    path = []
    x = np.random.random


class Simulation:
    def __init__(self, obstacles, walls, path, robotRange, robotFov, robotNumRays, rangeNoiseMag, bearingNoiseMag):
         
        self.obstacles = obstacles # obstacle locations (x, y)
        
        self.walls = walls # square wall endpoints (upper left(x1, y1), lower right(x2, y2))
        self.path = path
        self.robotRange = robotRange
        self.robotFov = robotFov
        self.robotNumRays = robotNumRays 
        self.noisyRange = lambda x: x + np.random.standard_normal() * rangeNoiseMag
        self.noisyBearing = lambda x: x + np.random.standard_normal() * bearingNoiseMag
        
    def wrapToTau(self, theta):
        return theta % 2 * np.pi

    def wrapToPi(self, theta):
        return self.wrapToTau(theta - np.pi) - np.pi

    def wallMeasurement(self, pose):
        # Unpack pose
        time, rx, ry, rTheta = pose

        # Generate endpoints of rays based on the robot's pose
        rayEndpoints = [] # (x, y)
        dTheta = self.robotFov / self.robotNumRays
        theta = self.wrapToTau(rTheta - self.robotFOV / 2)
        for i in range(self.robotNumRays):
            theta += dTheta
            endpointX = self.robotRange * np.cos(theta) + rx
            endpointY = self.robotRange * np.sin(theta) + ry
            rayEndpoints += [(endpointX, endpointY)]
        
        # Array of ranges to an intersection along each ray. If no intersection is detected, the max range is used
        ranges = []
        for rIndex in range(self.robotNumRays):
            for wall in self.walls:
                
        return ranges
    
    def landmarkMeasurement(self,pose):
        observedLandmarks = []

        time, rx, ry, rTheta = pose
        
        for (x,y) in self.obstacles:
            dx = rx - x
            dy = ry - y
            obstacleRange = (dx**2 + dy**2)**.5 
            if obstacleRange <= self.robotRange:
                obstacleBearing = np.arctan2(dy/dx)
                if self._isBearingInView(obstacleBearing, rTheta, self.robotFov):
                    observedLandmarks.append(
                      (x,y),self.noisyRange(obstacleRange), self.noisyBearing(obstacleBearing))
        return observedLandmarks
    
    def _isBearingInView(self,obstacleBearing, rTheta, rFov):
        return obstacleBearing >= rTheta and obstacleBearing <= (rTheta + rFov)%(2*np.pi)

    def getMeasurement(self):
        landmarkMeasurements = []
        wallsMeasurements = []
        for pose in self.path:
            landmarkMeasurements.append(self.landmarkMeasurement(pose))
            wallsMeasurements.append(self.wallMeasurement(pose))
        return landmarkMeasurements, wallsMeasurements



