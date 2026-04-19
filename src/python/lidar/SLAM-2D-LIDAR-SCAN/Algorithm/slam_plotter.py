import socket
import threading
import queue
import copy
import math
from Utils.ScanMatcher_OGBased import ScanMatcher
from Utils.OccupancyGrid import OccupancyGrid
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import json
import numpy as np
import matplotlib
import iceoryx2 as iox2
import quad_ipc
from system_logic import SystemLogic, to_event_id
import time
from gamepad_data import GamepadData

matplotlib.use("QtAgg")


pollHz = 100
pollCycle = 1.0 / pollHz
vel_stick = 20.0 # Feet per minute
stick_deadzone = 0.05
class GamepadSubscriber():
    def __init__(self):
        self.data = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
    
    def _loop(self):
        node = quad_ipc.make_node("gamepad_sub")
        service = (
            node.service_builder(iox2.ServiceName.new("GamepadData"))
            .publish_subscribe(GamepadData)
            .open_or_create()
        )
        sub = service.subscriber_builder().create()
        while True:
            tStart = time.monotonic()
            sample = sub.receive()
            if sample is not None:
                self.data = sample.payload()
            
            elapsed = time.monotonic() - tStart
            sleep_time = pollCycle - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    def get_velocity(self) -> dict:
        data = self.data
        if data is None:
            return {"vel_x": vel_x, "vel_y": vel_y}
        if abs(data.lx) > stick_deadzone:
            vel_x = data.lx * vel_stick
        else:
            vel_x = 0
        
        if abs(data.ly) > stick_deadzone:
            vel_y = data.ly * vel_stick
        else:
            vel_y = 0
        return {"vel_x": vel_x, "vel_y": vel_y}
# ==============================================================================
#  All units are FEET throughout (distances, map sizes, poses, grid size).
#
#  UDP changes vs your uploaded version:
#
#  1. socket.AFINET  ->  socket.AF_INET  (typo fix, was crashing immediately)
#
#  2. run_server no longer calls itself recursively at the end.
#
#  3. run_server now runs in a daemon background thread so main() is not
#     blocked.  Received scan dicts are placed onto a thread-safe Queue and
#     the main loop pulls from that queue instead of reading a JSON file.
#
#  4. processSensorData now accepts either a pre-loaded dict (offline mode,
#     for replaying a saved JSON) or a Queue (live UDP mode).  Pass
#     use_udp=True from main() to use the live path.
# ==============================================================================

UDP_HOST = '127.0.0.1'
UDP_PORT = 6000

# Thread-safe queue: the UDP server thread puts scan dicts here,
# the main/plotting thread reads from here.
scan_queue = queue.Queue()


def run_server():
    """
    UDP server — runs in a background daemon thread.
    Each received packet is a JSON-encoded scan entry dict.
    Decoded dicts are placed onto scan_queue for the SLAM loop to consume.

    FIX 1: AF_INET not AFINET
    FIX 2: no recursive self-call at the end
    FIX 3: runs as a thread so it doesn't block main()
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # FIX 1
    sock.bind((UDP_HOST, UDP_PORT))
    print(f"UDP server listening on {UDP_HOST}:{UDP_PORT}")

    while True:                                                # FIX 2: plain loop, no recursion
        try:
            data, address = sock.recvfrom(65507)
            scan_entry = json.loads(data.decode('utf-8'))
            scan_queue.put(scan_entry)
            sock.sendto(b'ack', address)
        except Exception as e:
            print(f"UDP server error: {e}")


def start_udp_server():
    """Start run_server in a background daemon thread."""    # FIX 3
    t = threading.Thread(target=run_server, daemon=True)
    t.start()
    return t


# ==============================================================================
# Particle filter / SLAM (unchanged logic, same as previous version)
# ==============================================================================

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

    def updateParticles(self, reading, count):
        for i in range(self.numParticles):
            self.particles[i].update(reading, count)

    def weightUnbalanced(self):
        self.normalizeWeights()
        variance = 0
        for i in range(self.numParticles):
            variance += (self.particles[i].weight - 1 / self.numParticles) ** 2
        print(f"  weight variance: {variance:.6f}")
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
        weights = np.array([p.weight for p in self.particles], dtype=float)
        tempParticles = [copy.deepcopy(p) for p in self.particles]
        resampledIdx = np.random.choice(
            np.arange(self.numParticles), self.numParticles, p=weights
        )
        for i in range(self.numParticles):
            self.particles[i] = copy.deepcopy(tempParticles[resampledIdx[i]])
            self.particles[i].weight = 1.0 / self.numParticles


class Particle:
    def __init__(self, ogParameters, smParameters):
        (initMapXLength, initMapYLength, initXY, unitGridSize,
         lidarFOV, lidarMaxRange, numSamplesPerRev, wallThickness) = ogParameters

        (scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid,
         moveRSigma, maxMoveDeviation, turnSigma,
         missMatchProbAtCoarse, coarseFactor) = smParameters

        og = OccupancyGrid(
            initMapXLength, initMapYLength, initXY, unitGridSize,
            lidarFOV, numSamplesPerRev, lidarMaxRange, wallThickness
        )
        sm = ScanMatcher(
            og, scanMatchSearchRadius, scanMatchSearchHalfRad,
            scanSigmaInNumGrid, moveRSigma, maxMoveDeviation,
            turnSigma, missMatchProbAtCoarse, coarseFactor
        )
        self.og = og
        self.sm = sm
        self.xTrajectory = []
        self.yTrajectory = []
        self.weight = 1.0

    def updateEstimatedPose(self, currentRawReading):
        estimatedReading = {
            'x':     self.prevMatchedReading['x'],
            'y':     self.prevMatchedReading['y'],
            'theta': self.prevMatchedReading['theta'],
            'range': currentRawReading['range'],
        }
        return estimatedReading, 0.0, None, None

    def getMovingTheta(self, matchedReading):
        if not self.xTrajectory:
            return None
        prevX, prevY = self.xTrajectory[-1], self.yTrajectory[-1]
        xMove = matchedReading['x'] - prevX
        yMove = matchedReading['y'] - prevY
        move = math.sqrt(xMove ** 2 + yMove ** 2)
        if move == 0:
            return None
        cos_val = max(-1.0, min(1.0, xMove / move))
        return math.acos(cos_val) if yMove >= 0 else -math.acos(cos_val)

    def update(self, reading, count):
        if count == 1:
            self.prevRawMovingTheta = None
            self.prevMatchedMovingTheta = None
            matchedReading = reading
            confidence = 1.0
        else:
            estimatedReading, estMovingDist, estMovingTheta, rawMovingTheta = \
                self.updateEstimatedPose(reading)
            matchedReading, confidence = self.sm.matchScan(
                estimatedReading, estMovingDist, estMovingTheta,
                count, matchMax=False
            )
            self.prevRawMovingTheta = rawMovingTheta
            self.prevMatchedMovingTheta = self.getMovingTheta(matchedReading)

        self.updateTrajectory(matchedReading)
        self.og.updateOccupancyGrid(matchedReading)
        self.prevMatchedReading = matchedReading
        self.prevRawReading = reading
        self.weight *= confidence

    def updateTrajectory(self, matchedReading):
        self.xTrajectory.append(matchedReading['x'])
        self.yTrajectory.append(matchedReading['y'])

    def plotParticle(self):
        plt.figure(figsize=(19.20, 19.20))
        plt.scatter(self.xTrajectory[0], self.yTrajectory[0], color='r', s=500)
        colors = iter(cm.rainbow(np.linspace(1, 0, len(self.xTrajectory) + 1)))
        for i in range(len(self.xTrajectory)):
            plt.scatter(self.xTrajectory[i], self.yTrajectory[i],
                        color=next(colors), s=35)
        plt.scatter(self.xTrajectory[-1], self.yTrajectory[-1],
                    color=next(colors), s=500)
        plt.plot(self.xTrajectory, self.yTrajectory)
        self.og.plotOccupancyGrid([-50, 50], [-50, 50], plotThreshold=False)


# ==============================================================================
# Live processing loop
# ==============================================================================

def processSensorData(pf, source, use_udp=False, human_is=False):
    gamepad = GamepadSubscriber()
    """
    source:
      - use_udp=False  ->  source is a dict  {timestamp: scan_entry, ...}
                           (offline replay from a saved JSON)
      - use_udp=True   ->  source is ignored; scans are pulled from scan_queue
                           as they arrive from the UDP server
    """
    count = 0
    potential_human = np.array((0, 2))
    human_markers = 0
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    img = None
    pos_dot = None
    heading_arrow = None

    def get_next_scan():
        """Return the next scan entry dict, blocking if in UDP mode."""
        if use_udp:
            return scan_queue.get()   # blocks until a scan arrives
        else:
            return None               # handled by the forloop below

    def process_one(scan_entry):
        # using above variables declared
        nonlocal count, img, pos_dot, heading_arrow
        nonlocal potential_human, human_markers
        count += 1  # increases how many frames we have made
        print(f"Frame {count}")

        pf.updateParticles(scan_entry, count)

        if pf.weightUnbalanced():
            pf.resample()
            print("  -> resampled")

        bestParticle = max(pf.particles, key=lambda p: p.weight)

        xRange = [-50, 50]
        yRange = [-50, 50]

        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.where(
                bestParticle.og.occupancyGridTotal > 0,
                bestParticle.og.occupancyGridVisited / bestParticle.og.occupancyGridTotal,
                0.5
            )

        xIdx, yIdx = bestParticle.og.convertRealXYToMapIdx(xRange, yRange)
        ogMap = ratio[yIdx[0]: yIdx[1], xIdx[0]: xIdx[1]]
        ogMap = np.flipud(1.0 - ogMap)

        if img is None:
            img = ax.imshow(
                ogMap,
                cmap='gray',
                vmin=0.0, vmax=1.0,
                extent=[xRange[0], xRange[1], yRange[0], yRange[1]],
                animated=True,
                origin='upper',
            )
            plt.show(block=False)
        else:
            img.set_data(ogMap)
            img.set_extent([xRange[0], xRange[1], yRange[0], yRange[1]])

        x_robot = bestParticle.xTrajectory[-1]
        y_robot = bestParticle.yTrajectory[-1]
        theta_robot = bestParticle.prevMatchedReading['theta']
        arrow_len = 1.5

        if pos_dot is not None:
            pos_dot.remove()
        if heading_arrow is not None:
            heading_arrow.remove()

        pos_dot = ax.plot(x_robot, y_robot, 'ro', markersize=8, zorder=5)[0]
        heading_arrow = ax.annotate(
            '',
            xy=(x_robot + arrow_len * math.cos(theta_robot),
                y_robot + arrow_len * math.sin(theta_robot)),
            xytext=(x_robot, y_robot),
            arrowprops=dict(arrowstyle='->', color='red', lw=2),
            zorder=5,
        )

        if human_is:
            # Create a grid of 20 x 20, and each unit is 1 feet of distance
            x_grid = np.arange(x_robot - 10, x_robot + 11, 1)
            y_grid = np.arange(y_robot - 10, y_robot + 11, 1)

            # is close Returns a boolean array where two arrays are element-wise equal within a tolerance.
            # compare both of x and y with a tolerance parameter of 0.5 to any grid line, centered around the robot
            on_grid = (
                np.isclose(x_robot, x_grid, atol=0.5).any() and
                np.isclose(y_robot, y_grid, atol=0.5).any()
            )

            if on_grid:
                if potential_human.size == 0:
                    add_marker = True
                else:
                    distances = np.linalg.norm(
                        potential_human - [x_robot, y_robot], axis=1)
                    add_marker = distances.min() >= 15  # only add if farther than 15 feet

                if add_marker:
                    ax.plot(x_robot, y_robot, 'D:g', markersize=8, zorder=4)
                    if potential_human.size == 0:
                        potential_human = np.array([[x_robot, y_robot]])
                    else:
                        potential_human = np.vstack(
                            [potential_human, [x_robot, y_robot]])
                    human_markers += 1
        
        ax.set_title(f"Live Map  —  Frame {count}")
        ax.set_xlabel("X (ft)")
        ax.set_ylabel("Y (ft)")
        ax.set_xlim(xRange[0], xRange[1])
        ax.set_ylim(yRange[0], yRange[1])
        ax.set_aspect('equal', adjustable='box')

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

    if use_udp:
        # Live mode: loop forever pulling scans from the queue
        print("Waiting for UDP scans...")
        try:
            while True:
                scan_entry = scan_queue.get()
                process_one(scan_entry)
        except KeyboardInterrupt:
            pass
    else:
        # Offline mode: replay a saved dict in timestamp order
        for key in sorted(source.keys()):
            process_one(source[key])

    plt.ioff()

    bestParticle = max(pf.particles, key=lambda p: p.weight)
    for particle in pf.particles:
        particle.plotParticle()
    bestParticle.plotParticle()

    plt.show()


def readJson(jsonFile):
    with open(jsonFile, 'r') as f:
        data = json.load(f)
    return data['map']


# ==============================================================================
# main
# ==============================================================================

def main():
    # ------------------------------------------------------------------
    # Map / sensor parameters — ALL IN FEET
    # ------------------------------------------------------------------
    initMapXLength = 100
    initMapYLength = 100
    unitGridSize = 0.25          # ft  (~3 inches)
    lidarFOV = math.radians(120)
    lidarMaxRange = 164.042       # ft  (50 m)
    wallThickness = 1.0           # ft

    scanMatchSearchRadius = 1.5
    scanMatchSearchHalfRad = 0.1
    scanSigmaInNumGrid = 2
    moveRSigma = 0.15
    maxMoveDeviation = 0.25
    turnSigma = 0.1
    missMatchProbAtCoarse = 0.15
    coarseFactor = 5

    
    # ------------------------------------------------------------------
    # Choose mode:
    #   use_udp = True   ->  receive live scans from sf45_collector over UDP
    #   use_udp = False  ->  replay a previously saved JSON file
    # ------------------------------------------------------------------
    use_udp = True

    if use_udp:
        # Start the UDP server in the background before building the filter,
        # so no packets are dropped while initialisation runs.
        start_udp_server()

        # Block until the very first scan arrives so we have initXY
        print("Waiting for first scan to initialise map...")
        first_scan = scan_queue.get()
        scan_queue.put(first_scan)   # put it back so the SLAM loop sees it too

        numSamplesPerRev = len(first_scan['range'])
        initXY = first_scan
        sensorData = None
    else:
        sensorData = readJson("DataSet/PreprocessedData/SF45_Live_Data.json")
        firstKey = sorted(sensorData.keys())[0]
        numSamplesPerRev = len(sensorData[firstKey]['range'])
        initXY = sensorData[firstKey]

    print(f"numSamplesPerRev = {numSamplesPerRev}")
    print(f"lidarFOV = {math.degrees(lidarFOV):.1f} deg  |  "
          f"lidarMaxRange = {lidarMaxRange} ft  |  "
          f"gridSize = {unitGridSize} ft")

    numParticles = 10

    ogParameters = [
        initMapXLength, initMapYLength, initXY,
        unitGridSize, lidarFOV, lidarMaxRange,
        numSamplesPerRev, wallThickness,
    ]
    smParameters = [
        scanMatchSearchRadius, scanMatchSearchHalfRad,
        scanSigmaInNumGrid, moveRSigma, maxMoveDeviation,
        turnSigma, missMatchProbAtCoarse, coarseFactor,
    ]

    pf = ParticleFilter(numParticles, ogParameters, smParameters)
    processSensorData(pf, sensorData, use_udp=use_udp)


if __name__ == '__main__':
    main()
