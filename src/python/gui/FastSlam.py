import copy
import math
from ScanMatcher_OGBased import ScanMatcher
from OccupancyGrid import OccupancyGrid
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import json
import numpy as np
import matplotlib
matplotlib.use("QtAgg")


class ParticleFilter:
    def __init__(self, numParticles, ogParameters, smParameters):
        self.numParticles = numParticles
        self.particles = []
        self.initParticles(ogParameters, smParameters)
        self.step = 0
        self.prevMatchedReading = None
        self.prevRawReading = None
        self.particlesTrajectory = []

    def initParticles(self, ogParameters, smParameters):
        for i in range(self.numParticles):
            p = Particle(ogParameters, smParameters)
            self.particles.append(p)

    # FIX: Added 'motion' parameter here
    def updateParticles(self, reading, count, motion=None):
        for i in range(self.numParticles):
            self.particles[i].update(reading, count, motion)

    def weightUnbalanced(self):
        self.normalizeWeights()
        variance = 0
        for i in range(self.numParticles):
            variance += (self.particles[i].weight - 1 / self.numParticles) ** 2
        
        threshold = (
            ((self.numParticles - 1) / self.numParticles) ** 2
            + (self.numParticles - 1.000000000000001) *
            (1 / self.numParticles) ** 2
        )
        return variance > threshold

    def normalizeWeights(self):
        weightSum = sum(p.weight for p in self.particles)
        if weightSum == 0:
            for p in self.particles:
                p.weight = 1.0 / self.numParticles
        else:
            for p in self.particles:
                p.weight /= weightSum

    def resample(self):
        import copy
        import numpy as np
        weights = np.array([p.weight for p in self.particles], dtype=float)
        idx = np.random.choice(self.numParticles, self.numParticles, p=weights)
        new_particles = []
        used = set()
        for i in idx:
            if i not in used:
                used.add(i)
                new_particles.append(self.particles[i])
            else:
                new_particles.append(copy.deepcopy(self.particles[i]))
        for p in new_particles:
            p.weight = 1.0 / self.numParticles
        self.particles = new_particles


class Particle:
    def __init__(self, ogParameters, smParameters):
        from OccupancyGrid import OccupancyGrid
        from ScanMatcher_OGBased import ScanMatcher
        import math
        
        (initMapXLength, initMapYLength, initXY, unitGridSize,
         lidarFOV, lidarMaxRange, numSamplesPerRev, wallThickness) = ogParameters

        (scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid,
         moveRSigma, maxMoveDeviation, turnSigma,
         missMatchProbAtCoarse, coarseFactor) = smParameters

        self.og = OccupancyGrid(
            initMapXLength, initMapYLength, initXY, unitGridSize,
            lidarFOV, numSamplesPerRev, lidarMaxRange, wallThickness
        )
        self.sm = ScanMatcher(
            self.og, scanMatchSearchRadius, scanMatchSearchHalfRad,
            scanSigmaInNumGrid, moveRSigma, maxMoveDeviation,
            turnSigma, missMatchProbAtCoarse, coarseFactor
        )
        self.xTrajectory = []
        self.yTrajectory = []
        self.weight = 1.0
        self.prevMatchedReading = None
        self.prevRawReading = None

    # FIX: This now uses the motion data (speed/dt) from your dashboard
    def updateEstimatedPose(self, currentRawReading, motion):
        import math
        speed = motion['speed']
        strafe = motion.get('strafe', 0.0)
        orientation = motion['orientation']
        dt = motion['dt']

        # Calculate displacement based on speed and heading
        dx = speed  * math.cos(orientation) * dt - strafe * math.sin(orientation) * dt
        dy = speed  * math.sin(orientation) * dt + strafe * math.cos(orientation) * dt


        estimatedReading = {
            'x':     self.prevMatchedReading['x'] + dx,
            'y':     self.prevMatchedReading['y'] + dy,
            'theta': orientation,   
            'range': currentRawReading['range'],
        }
        estMovingDist = math.hypot(dx, dy)
        estMovingTheta = math.atan2(dy, dx) if estMovingDist > 1e-6 else None
        return estimatedReading, estMovingDist, estMovingTheta, estMovingTheta

    def update(self, reading, count, motion=None):
        if count == 1:
            matchedReading, confidence = reading, 1.0
        elif motion is None:
            # Fallback if no motion is provided
            matchedReading = self.prevMatchedReading.copy()
            matchedReading['range'] = reading['range']
            confidence = 1.0
        else:
            estimatedReading, estMovingDist, estMovingTheta, rawMovingTheta = \
                self.updateEstimatedPose(reading, motion)
            
            matchedReading, confidence = self.sm.matchScan(
                estimatedReading, estMovingDist, estMovingTheta,
                count, matchMax=False
            )

        self.xTrajectory.append(matchedReading['x'])
        self.yTrajectory.append(matchedReading['y'])
        self.og.updateOccupancyGrid(matchedReading)
        self.prevMatchedReading = matchedReading
        self.weight *= confidence

    def updateTrajectory(self, matchedReading):
        x, y = matchedReading['x'], matchedReading['y']
        self.xTrajectory.append(x)
        self.yTrajectory.append(y)

    def plotParticle(self):
        plt.figure(figsize=(19.20, 19.20))
        plt.scatter(self.xTrajectory[0], self.yTrajectory[0], color='r', s=500)
        colors = iter(cm.rainbow(np.linspace(1, 0, len(self.xTrajectory) + 1)))
        for i in range(len(self.xTrajectory)):
            plt.scatter(
                self.xTrajectory[i], self.yTrajectory[i], color=next(colors), s=35)
        plt.scatter(
            self.xTrajectory[-1], self.yTrajectory[-1], color=next(colors), s=500)
        plt.plot(self.xTrajectory, self.yTrajectory)
        self.og.plotOccupancyGrid([-13, 20], [-25, 7], plotThreshold=False)


def processSensorData(pf, sensorData, plotTrajectory=True):
    # gtData = readJson("../DataSet/PreprocessedData/intel_corrected_log") #########   For Debug Only  #############
    count = 0
    plt.ion()
    fig, ax = plt.subplots()
    img = None
    pos_dot = None
    heading_arrow = None
    traj_line = None

    for key in sorted(sensorData.keys()):
        count += 1
        print(count)
        pf.updateParticles(sensorData[key], count)
        if pf.weightUnbalanced():
            pf.resample()
            print("resample")

        # plt.figure(figsize=(19.20, 19.20))
        maxWeight = -1
        for particle in pf.particles:
            if maxWeight < particle.weight:
                maxWeight = particle.weight
                bestParticle = particle
            #    plt.plot(particle.xTrajectory, particle.yTrajectory)

        xRange, yRange = [-25, 25], [-25, 25]
        ogMap = bestParticle.og.occupancyGridVisited / bestParticle.og.occupancyGridTotal
        xIdx, yIdx = bestParticle.og.convertRealXYToMapIdx(xRange, yRange)
        ogMap = ogMap[yIdx[0]: yIdx[1], xIdx[0]: xIdx[1]]
        ogMap = np.flipud(1 - ogMap)
        if img is None:
            img = ax.imshow(
                ogMap,
                cmap='gray',
                extent=[xRange[0], xRange[1], yRange[0], yRange[1]],
                animated=True
            )
            plt.show(block=False)
        else:
            img.set_data(ogMap)
            img.set_extent([xRange[0], xRange[1], yRange[0], yRange[1]])
            # draw current sensor/camera position and heading
            x_cam = bestParticle.xTrajectory[-1]
            y_cam = bestParticle.yTrajectory[-1]
            theta_cam = bestParticle.prevMatchedReading['theta']

        ax.set_title(f"Live Map Frame {count}")
        ax.set_xlabel("X (ft)")
        ax.set_ylabel("Y (ft)")
        ax.set_xlim(-25, 25)
        ax.set_ylim(-25, 25)
        ax.set_aspect('equal', adjustable='box')
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

    plt.ioff()
    plt.show()
    # if count == 100:
    #     break
    maxWeight = 0
    for particle in pf.particles:
        particle.plotParticle()
        if maxWeight < particle.weight:
            maxWeight = particle.weight
            bestParticle = particle
    bestParticle.plotParticle()


def readJson(jsonFile):
    with open(jsonFile, 'r') as f:
        input = json.load(f)
        return input['map']


def main():
    initMapXLength, initMapYLength, unitGridSize, lidarFOV, lidarMaxRange = 50, 100, 0.5, np.pi, 100  # in Feet
    scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid, wallThickness, moveRSigma, maxMoveDeviation, turnSigma, \
        missMatchProbAtCoarse, coarseFactor = 1.4, 0.25, 2, 5 * \
        unitGridSize, 0.1, 0.25, 0.3, 0.15, 5
    sensorData = readJson("DataSet/PreprocessedData/SF45_Live_Data.json")
    # Get how many points per revolution
    numSamplesPerRev = len(sensorData[list(sensorData)[0]]['range'])
    initXY = sensorData[sorted(sensorData.keys())[0]]
    numParticles = 10
    ogParameters = [initMapXLength, initMapYLength, initXY, unitGridSize,
                    lidarFOV, lidarMaxRange, numSamplesPerRev, wallThickness]
    smParameters = [scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid, moveRSigma, maxMoveDeviation, turnSigma,
                    missMatchProbAtCoarse, coarseFactor]
    pf = ParticleFilter(numParticles, ogParameters, smParameters)
    processSensorData(pf, sensorData, plotTrajectory=True)


if __name__ == '__main__':
    main()