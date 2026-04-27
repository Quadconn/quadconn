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
import time
import os
from datetime import datetime

matplotlib.use("QtAgg")
# View modes 
# View mode (toggled by Ctrl+= / Ctrl+-)
VIEW_LIVE = 'live'        # robot-centered 100x100 window
VIEW_HISTORY = 'history'  # auto-fit to the whole explored grid
view_mode = VIEW_LIVE     # mutable at runtime via keypress

LIVE_VIEW_HALF = 50.0     # ft; window is 2*this on each axis

# --- Human Detection Variables ----
human_present = None # Change this value in function if human was detected

METERS_TO_FEET = 3.28084
IDLE_EPS = 1e-3

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

UDP_HOST = '100.97.181.114'
UDP_PORT = 6000
PLOT_EVERY = 3
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

    def updateParticles(self, reading, count, motion=None):
        for i in range(self.numParticles):
            self.particles[i].update(reading, count, motion)

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
        # it only deepcopies the duplicates not new particles
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
        self.prevRawMovingTheta = None
        self.prevMatchedMovingTheta = None

    def updateEstimatedPose(self, currentRawReading, motion):
        speed = motion['speed']
        orientation = motion['orientation']
        dt = motion['dt']

        # Scalar forward speed + absolute heading -> world-frame displacement.
        dx = speed * math.cos(orientation) * dt
        dy = speed * math.sin(orientation) * dt

        estimatedReading = {
            'x':     self.prevMatchedReading['x'] + dx,
            'y':     self.prevMatchedReading['y'] + dy,
            'theta': orientation,   # use the measured orientation as our heading prior
            'range': currentRawReading['range'],
        }
        estMovingDist = math.hypot(dx, dy)
        estMovingTheta = math.atan2(dy, dx) if estMovingDist > 1e-6 else None
        return estimatedReading, estMovingDist, estMovingTheta, estMovingTheta

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

    def update(self, reading, count, motion= None):
        if count == 1:
            matchedReading, confidence = reading, 1.0

        elif motion is None:
            matchedReading = {
                'x': self.prevMatchedReading['x'],
                'y': self.prevMatchedReading['y'],
                'theta': self.prevMatchedReading['theta'],
                'range': reading['range'],
            }
            confidence = 1.0

        else:
            estimatedReading, estMovingDist, estMovingTheta, rawMovingTheta = \
                self.updateEstimatedPose(reading, motion)
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

def processSensorData(pf, source, use_udp=False):
    """
    source:
      - use_udp=False  ->  source is a dict  {timestamp: scan_entry, ...}
                           (offline replay from a saved JSON)
      - use_udp=True   ->  source is ignored; scans are pulled from scan_queue
                           as they arrive from the UDP server
    """
    last_scan_time = None
    count = 0
    potential_human = np.empty((0, 2))
    human_markers = 0
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    def save_map_image():
        """Save the full explored grid as a JPEG to the working directory."""
        if not pf.particles:
            return
        best = max(pf.particles, key=lambda p: p.weight)
        og = best.og

        xRange = list(og.mapXLim)
        yRange = list(og.mapYLim)
        xIdx, yIdx = og.convertRealXYToMapIdx(xRange, yRange)
        visited = og.occupancyGridVisited[yIdx[0]:yIdx[1], xIdx[0]:xIdx[1]]
        total   = og.occupancyGridTotal  [yIdx[0]:yIdx[1], xIdx[0]:xIdx[1]]
        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.where(total > 0, visited / total, 0.5)
        ogMap = np.flipud(1.0 - ratio)

        save_fig, save_ax = plt.subplots(figsize=(10, 10))
        save_ax.imshow(
            ogMap, cmap='gray', vmin=0.0, vmax=1.0,
            extent=[xRange[0], xRange[1], yRange[0], yRange[1]],
            origin='upper',
        )

        save_ax.set_title("Map — full explored area")
        save_ax.set_xlabel("X (ft)")
        save_ax.set_ylabel("Y (ft)")
        save_ax.set_aspect('equal', adjustable='box')

        # Filename: Map_DD_MM_YYYY_HHMM.jpg
        stamp = datetime.now().strftime("%d_%m_%Y_%H%M")
        filename = os.path.join(os.getcwd(), f"Map_{stamp}.jpg")
        save_fig.savefig(filename, dpi=150, format='jpg', bbox_inches='tight')
        plt.close(save_fig)
        print(f"Saved map to {filename}")

    def on_key(event):
        """Ctrl+= -> live view, Ctrl+- -> history view."""
        global view_mode
        if event.key in ('ctrl+=', 'ctrl++'):
            view_mode = VIEW_LIVE
            print("View: LIVE (robot-centered 100x100)")
        elif event.key == 'ctrl+-':
            view_mode = VIEW_HISTORY
            print("View: HISTORY (full explored map)")

    def on_close(event):
        """Save map when the matplotlib window is closed."""
        print("Window closed — saving map...")
        save_map_image()

    fig.canvas.mpl_connect('key_press_event', on_key)
    fig.canvas.mpl_connect('close_event', on_close)    

    img = None
    pos_dot = None

    def get_next_scan():
        """Return the next scan entry dict, blocking if in UDP mode."""
        if use_udp:
            return scan_queue.get()   # blocks until a scan arrives
        else:
            return None               # handled by the forloop below

    def process_one(scan_entry):
        # using above variables declared
        nonlocal count, img, pos_dot, last_scan_time
        nonlocal potential_human, human_markers
        human_is = human_present
        count += 1  # increases how many frames we have made
        print(f"Frame {count}")
        now = time.monotonic()
        
        dt = (now - last_scan_time) if last_scan_time is not None else 0.0
        last_scan_time = now
        
        speed = scan_entry.get('speed', 0.0) * METERS_TO_FEET                # already in ft/sec
        orientation = scan_entry.get('theta', 0.0)    # rad

        isMoving = abs(speed) > IDLE_EPS

        if isMoving:
            # Convert scalar forward speed + absolute heading into a world-frame
            # displacement prediction over dt.
            motion = {
                'speed':       speed,
                'orientation': orientation,
                'dt':          dt,
            }
        else:
            motion = None
        
        pf.updateParticles(scan_entry, count, motion)

        if pf.weightUnbalanced():
            pf.resample()
            print("  -> resampled")

        bestParticle = max(pf.particles, key=lambda p: p.weight)

        # ---- plotting only every PLOT_EVERY frames ----
        if count % PLOT_EVERY != 0:
            return
        
# ---- Determine view range based on current mode ----
        x_robot_live = bestParticle.xTrajectory[-1]
        y_robot_live = bestParticle.yTrajectory[-1]

        og = bestParticle.og
        if view_mode == VIEW_LIVE:
            # Robot-centered 100x100 ft window. The grid may extend beyond
            # these bounds — we just slice the part we care about.
            xRange = [x_robot_live - LIVE_VIEW_HALF, x_robot_live + LIVE_VIEW_HALF]
            yRange = [y_robot_live - LIVE_VIEW_HALF, y_robot_live + LIVE_VIEW_HALF]
            # Clip to grid bounds so slicing doesn't run off the edge.
            xRange = [max(xRange[0], og.mapXLim[0]), min(xRange[1], og.mapXLim[1])]
            yRange = [max(yRange[0], og.mapYLim[0]), min(yRange[1], og.mapYLim[1])]
        else:  # VIEW_HISTORY
            # Show the entire explored grid.
            xRange = list(og.mapXLim)
            yRange = list(og.mapYLim)

        xIdx, yIdx = og.convertRealXYToMapIdx(xRange, yRange)
        visited = og.occupancyGridVisited[yIdx[0]:yIdx[1], xIdx[0]:xIdx[1]]
        total   = og.occupancyGridTotal  [yIdx[0]:yIdx[1], xIdx[0]:xIdx[1]]
        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.where(total > 0, visited / total, 0.5)
        ogMap = np.flipud(1.0 - ratio)

        if img is None:
            img = ax.imshow(
                ogMap, cmap='gray', vmin=0.0, vmax=1.0,
                extent=[xRange[0], xRange[1], yRange[0], yRange[1]],
                animated=True, origin='upper',
            )
            plt.show(block=False)
        else:
            img.set_data(ogMap)
            img.set_extent([xRange[0], xRange[1], yRange[0], yRange[1]])

        # Robot marker 
        if pos_dot is not None:
            pos_dot.remove()

        pos_dot = ax.plot(x_robot_live, y_robot_live, 'ro', markersize=8, zorder=5)[0]

        # Human markers (unchanged)
        if human_is:
            if potential_human.size == 0:
                nearby = False
            else:
                dx = np.abs(potential_human[:, 0] - x_robot_live)
                dy = np.abs(potential_human[:, 1] - y_robot_live)
                nearby = (np.maximum(dx, dy) <= 15.0).any()

            if not nearby:
                ax.plot(x_robot_live, y_robot_live, marker='D', color='g',
                        markersize=8, linestyle='none', zorder=4)
                if potential_human.size == 0:
                    potential_human = np.array([[x_robot_live, y_robot_live]])
                else:
                    potential_human = np.vstack([potential_human, [x_robot_live, y_robot_live]])
                human_markers += 1

        mode_label = "Focused Mapping" if view_mode == VIEW_LIVE else "Full Map"
        ax.set_title(f"{mode_label}  —  Frame {count}")
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
        print("Keys: Ctrl+= = live view, Ctrl+- = full history view")
        try:
            while True:
                scan_entry = scan_queue.get()
                process_one(scan_entry)
        except KeyboardInterrupt:
            print("\nCtrl-C received — saving map...")
            save_map_image()    
    else:
        # Offline mode: replay a saved dict in timestamp order
        try:
            for key in sorted(source.keys()):
                process_one(source[key])

        except KeyboardInterrupt:
            print("\nCtrl-C received — saving map...")
            save_map_image()
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
    unitGridSize = 0.5          # ft  (~6 inches)
    lidarFOV = math.radians(120)
    lidarMaxRange = 164.042       # ft  (50 m)
    wallThickness = 1.0           # ft

    scanMatchSearchRadius = 1.0
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
        while True:
            try:
                first_scan = scan_queue.get()
                break
            except queue.Empty():
                continue
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

    numParticles = 8

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
