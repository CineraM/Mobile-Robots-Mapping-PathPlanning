# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math, copy, json, os
#######################################################
# Creates Robot
#######################################################
robot = Robot()
#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())
#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)
#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()
# print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
# print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')
#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera_front = robot.getDevice('cameraFront')
camera_front.enable(timestep)
camera_front.recognitionEnable(timestep)
camera_right = robot.getDevice('cameraRight')
camera_right.enable(timestep)
camera_right.recognitionEnable(timestep)
camera_rear = robot.getDevice('cameraRear')
camera_rear.enable(timestep)
camera_rear.recognitionEnable(timestep)
camera_left = robot.getDevice('cameraLeft')
camera_left.enable(timestep)
camera_left.recognitionEnable(timestep)
#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)
#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)
################# HELPER FUNCTIONS #################
def getLidar():
    image = lidar.getRangeImage()
    ret = []
    ret.append(image[0]*toIn - half_of_robot)   # front
    ret.append(image[270]*toIn - half_of_robot) # left
    ret.append(image[90]*toIn - half_of_robot)  # right
    ret.append(image[180]*toIn - half_of_robot) # back
    
    return ret

# set speed to both motors, input in Inches
def setSpeedIPS(vl, vr):
    vl /= w_r
    vr /= w_r
    leftMotor.setVelocity(vl)
    rightMotor.setVelocity(vr)

# calculates distance after task
# For this lab only calculate when abs(vl) = abs(vr)
def distAfterTask(vl, vr):
    vl = round(vl, 8)
    vr = round(vr, 8)
    
    if vl == vr:
        return 0.032*vl
    if vl == -vr or math.isnan(vl):
        return 0
    return 0

# returns the decorders values
def getPositionSensors():
    return leftposition_sensor.getValue(), rightposition_sensor.getValue()

# return the imu reading in degrees instead of radians
def imuCleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)
################# HELPER FUNCTIONS #################

# global variables
distBtwWhe = 2.28
dmid = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2
pi = math.pi
half_of_robot = 0.037*39.3701 
toIn = 39.3701
prev_l, prev_r = getPositionSensors()
#########################################################
# Robot and Maze classes
class mazeMap:
    def __init__(self, n=0, tiles = [], graph = {}, grid = []):
        self.n = n 
        self.tiles = tiles
        self.graph = graph
        self.grid = grid

    # code that generates the 4 points for all tiles, 16 tiles
    # generates top left, top right, bottom left, bottom right points of an nxn grid
    def generateTiles(self):
        y = 20
        for i in range(self.n):
            x = -20
            for j in range(self.n):
                self.tiles.append([[x, y], [x+10, y], [x, y-10], [x+10, y-10]])
                x+=10
            y-=10

    # bottom left, top right, robot
    def updateTile(self, pose):
        # up, down, left, right instead looking though all the tiles
        
        cur_tile = pose.tile-1
        n = MAZE.n
        # up, left, right, down
        possible_tiles = [cur_tile-n, cur_tile-1, cur_tile+1, cur_tile+n]
        
        for i in possible_tiles:
            tl = self.tiles[i][0]
            br = self.tiles[i][3]
            x, y = pose.x, pose.y

            if x > tl[0] and x < br[0]:
                if y < tl[1] and y > br[1]:
                    return i+1
        return -1
    # the search space is extremly small, this will not affect performance 8x8
    # exaustive search just in case 
        # for i in range(len(self.tiles)):
        #     tl = self.tiles[i][0]
        #     br = self.tiles[i][3]
        #     x, y = pose.x, pose.y

        #     if x > tl[0] and x < br[0]:
        #         if y < tl[1] and y > br[1]:
        #             print(i)
        #             return i+1
        # return -1
    
    def generateGrid(self):
        for i in range(self.n):
            temp = [] 
            for j in range(self.n):
                temp.append(0)
            self.grid.append(temp)
    
    def updateGrid(self, tile):
        i = tile//self.n
        j = tile%self.n
        self.grid[i][j] = 1

    def bfs_queue_helper(self, paths, queue, nodes, cur_node):
        for node in nodes:
            if node != "wall": queue.append(node)
            if node not in paths:
                new_path = copy.deepcopy(paths[cur_node])
                new_path.append(node)
                paths[node] = new_path

        return paths, queue 

    def bfs(self, start_node, search_node):
        visited = [] # visited and path may be the same for bfs, test later!
        paths = {start_node: [start_node]}

        # attach to path once the node is explored
        queue = [start_node]
        # use a list as a queue
        # pop top of the queue and add neighbors
        while len(queue) > 0:
            cur_node = queue.pop(0)
            if cur_node not in visited:
                visited.append(cur_node)
                paths, queue = self.bfs_queue_helper(paths, queue, self.graph[cur_node], cur_node)

            if search_node == cur_node:
                return paths[search_node]

class RobotPose:
    def __init__(self, x, y, tile, theta):
        self.x = x
        self.y = y
        self.tile = tile
        self.theta = theta

    #print the grid & robot pose
    def printRobotPose(self, maze):
        print(f'x: {self.x:.2f}\ty: {self.y:.2f}\ttile: {self.tile}\ttheta: {self.theta:.2f}')
        for list in maze.grid:
            print("\t" + str(list))
        print("-----------------------------------------------")

    # Update pose of robot and grid, updates if a tile is found
    def updatePose(self, MAZE):
        global prev_l, prev_r
        self.printRobotPose(MAZE)
        cur_l, cur_r = getPositionSensors()
        vl = (cur_l-prev_l)/0.032   # 32 ms 
        vr = (cur_r-prev_r)/0.032
        imu_reading = imuCleaner(imu.getRollPitchYaw()[2])
        dist = distAfterTask(vl*w_r, vr*w_r)
        self.theta = imu_reading
        
        prev_l = cur_l
        prev_r = cur_r

        if imu_reading < 94 and imu_reading > 86:
            self.y += dist
        elif imu_reading < 184 and imu_reading > 176:
            self.x -= dist
        elif imu_reading < 274 and imu_reading > 266:
            self.y -= dist
        elif imu_reading <= 360 and imu_reading > 356 or imu_reading < 4 and imu_reading >= 0:
            self.x += dist

        tile = MAZE.updateTile(self)
        if tile != -1: 
            self.tile = tile
            MAZE.updateGrid(tile-1)

# initialize map and robot pose
MAZE = mazeMap(n=8)
MAZE.generateTiles()
MAZE.generateGrid()

ROBOT_POSE = RobotPose(15.0, -25.0, 36, 90)
MAZE.updateGrid(ROBOT_POSE.tile-1)

########################## Motion logic ######################## 
# 5.024 = max speed in in per second
def straightMotionD(d):
    v = 5.024
    is_neg = False
    if d < 0:
        is_neg = True
        d = abs(d)

    time = d/v  # 5.024 = v*r ==> max linear speed
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            setSpeedIPS(0,0)
            # ROBOT_POSE.updatePose(MAZE)
            # ROBOT_POSE.printRobotPose(MAZE)
            break
        if is_neg:
            setSpeedIPS(-v, -v)
        else:
            setSpeedIPS(v, v)
        # ROBOT_POSE.updatePose(MAZE)
        # ROBOT_POSE.printRobotPose(MAZE)

# assume angle is in radians
def rotationInPlace(direction, angle, v):
    s = angle*dmid
    time = s/v
    s_time = robot.getTime()

    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            # ROBOT_POSE.updatePose(MAZE)
            # ROBOT_POSE.printRobotPose(MAZE)
            break 
        if direction == "left":
            setSpeedIPS(-v, v)
        else:
            setSpeedIPS(v, -v)
            
        # ROBOT_POSE.updatePose(MAZE)
        # ROBOT_POSE.printRobotPose(MAZE)

def circularMotion(vr=3, direction="left", R=10, angle=pi/2):
    omega = vr/(R+dmid)
    vl = omega*(R-dmid)
    v = (vr+vl)/2
    s = (angle) * R
    time = s/v
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            setSpeedIPS(0,0)
            break 
        setSpeedIPS(0,0)
        if direction == "right":
            setSpeedIPS(vr,vl)
        else:
            setSpeedIPS(vl,vr)
########################## Motion logic ######################## 

def rotateUntilAngle(angle):
    while robot.step(timestep) != -1:
        setSpeedIPS(0.8, -0.8)
        theta = imuCleaner(imu.getRollPitchYaw()[2])

        if angle == 0:
            if theta <= 0.3:
                setSpeedIPS(0, 0)
                break
            elif theta >= 359.7 and theta <=360:
                setSpeedIPS(0, 0)
                break
        else:
            if theta <= angle+0.3 and theta >= angle-0.3:
                setSpeedIPS(0, 0)
                break

def loadGraph():
    path = os.getcwd()
    path = os.path.dirname(path) + "/graph.json" 

    with open(path, "r") as fp:
        MAZE.graph = json.load(fp)
    print("Graph successfully loaded!")
    print("Graph contents: (Edges format: Up, Left, Right, Down)")
    for i in range(4):
        for j in range(4):
            s = str(i) + ',' + str(j)
            if s in MAZE.graph:
                print(f'node: {s} , edges: {MAZE.graph[s]}')
    print("--------------------------------------------------")

def bfsToList(bfs):
    new_list = []
    for node in bfs:
        coma_indx = node.find(',')
        i = int(node[:coma_indx])
        j = int(node[coma_indx+1:])
        new_list.append([i, j])
    return new_list

def firstTheta(first, second):

    if first[1] == second[1]: 
        if first[0] > second[0]:
            return 90
        elif first[0] < second[0]:
            return 270
    elif first[0] == second[0]: 
        if first[1] > second[1]:
            return 180
        elif first[1] < second[1]:
            return 0


motion_theta = 90
def forwardHelper(a, b, c):
    if motion_theta == 0:
        if a[0] == b[0] and a[0] == c[0]:
            return True
    elif motion_theta == 90:
        if a[1] == b[1] and a[1] == c[1]:
            return True
    elif motion_theta == 180:
        if a[0] == b[0] and a[0] == c[0]:
            return True
    elif motion_theta == 270:
        if a[1] == b[1] and a[1] == c[1]:
            return True
    return False

# i, j
def quarterCircleHelper(a, b, c):
    global motion_theta
    if motion_theta == 0:
        # going right
        if a[0] > c[0]:
            motion_theta = 90
            return "ql"
        else:
            motion_theta = 270
            return "qr"

    elif motion_theta == 90:
        # going up
        if a[1] > c[1]:
            motion_theta = 180
            return "ql"
        else:
            motion_theta = 0
            return "qr"
        
    elif motion_theta == 180:
        # going left
        if a[0] > c[0]:
            motion_theta = 90
            return "qr"
        else:
            motion_theta = 270
            return "ql"
    elif motion_theta == 270:
        # going down
        if a[1] > c[1]:
            motion_theta = 180
            return "qr"
        else:
            motion_theta = 0
            return "ql"
        
    return False

# hl, hr
def halfCircleHelper(a, b, c, d):
    global motion_theta

    if motion_theta == 0:
        # going right
        if a[0] > c[0]:
            # motion_theta = 90
            if c[1] > d[1] and c[0] == d[0]:
                motion_theta = 180
                return "hl"
        else:
            # motion_theta = 270
            if c[1] > d[1] and c[0] == d[0]:
                motion_theta = 180
                return "hr"

    elif motion_theta == 90:
        # going up
        if a[1] > c[1]:
            # motion_theta = 180
            if c[0] < d[0] and c[1] == d[1]:
                motion_theta = 270
                return "hl"
        else:
            if c[0] < d[0] and c[1] == d[1]:
                motion_theta = 270
                return "hr"
        
    elif motion_theta == 180:
        # going left
        if a[0] > c[0]:
            # motion_theta = 90
            if c[1] < d[1] and c[0] == d[0]:
                motion_theta = 0
                return "hr"
        else:
            motion_theta = 270
            if c[1] < d[1] and c[0] == d[0]:
                motion_theta = 0
                return "hl"
            
    elif motion_theta == 270:
        # going down
        if a[1] > c[1]:
            if c[0] > d[0] and c[1] == d[1]:
                motion_theta = 90
                return "hr"
        else:
            if c[0] > d[0] and c[1] == d[1]:
                motion_theta = 90
                return "hl"
        
    return False


def rotationHelper(a, b):
    global motion_theta

    if motion_theta == 0:
        if a[0] > b[0]:
            motion_theta = 270
            return "ir"
        elif a[0] < b[0]:
            motion_theta = 90
            return "il"
        
    elif motion_theta == 90:
        if a[1] > b[1]:
            motion_theta = 180
            return "il"
        elif a[1] < b[1]:
            motion_theta = 0
            return "ir"
        
    elif motion_theta == 180:
        if a[0] > b[0]:
            motion_theta = 270
            return "il"
        elif a[0] < b[0]:
            motion_theta = 90
            return "ir"
        
    elif motion_theta == 270:
        if a[1] > b[1]:
            motion_theta = 180
            return "ir"
        elif a[1] < b[1]:
            motion_theta = 0
            return "il"
        
    return False



# f == forward 10, ql = quarter circle left, qr = quarter circle right
# hl = half circle left, hr = half circle right
# il, ir =  rotation in place left or right
def generateMotions(waypoints):
    motions = []

    for x in range(len(waypoints)):
        # print(waypoints)
        length = len(waypoints) 
        if length <= 0:
            break
        elif length == 1:
            waypoints.pop(0)

        elif length == 2:
            motions.append("f")
            waypoints.pop(0)
            waypoints.pop(0)

        elif length == 3:
            a = waypoints[0]
            b = waypoints[1]
            c = waypoints[2]

            is_forward = forwardHelper(a, b, c)
            if is_forward:
                waypoints.pop(0)
                motions.append("f")
                continue

            q_circle = quarterCircleHelper(a,b,c)
            if q_circle != False:
                motions.append(q_circle)
                waypoints.pop(0)
                waypoints.pop(0)
                continue
        elif length > 3:
            a = waypoints[0]
            b = waypoints[1]
            c = waypoints[2]
            d = waypoints[3]

            is_forward = forwardHelper(a, b, c)
            if is_forward:
                waypoints.pop(0)
                motions.append("f")
                continue

            h_circle = halfCircleHelper(a,b,c,d)
            if h_circle != False:
                motions.append(h_circle)
                waypoints.pop(0)
                waypoints.pop(0)
                waypoints.pop(0)

                if len(waypoints) >= 2:
                    rotate = rotationHelper(waypoints[0], waypoints[1])
                    if rotate != False: motions.append(rotate)
                continue

            q_circle = quarterCircleHelper(a,b,c)
            if q_circle != False:
                motions.append(q_circle)
                waypoints.pop(0)
                waypoints.pop(0)

                if len(waypoints) >= 2:
                    rotate = rotationHelper(waypoints[0], waypoints[1])
                    if rotate != False: motions.append(rotate)
                continue

    return motions

# f == forward 10, ql = quarter circle left, qr = quarter circle right
# hl = half circle left, hr = half circle right
# il, ir =  rotation in place left or right 
def runMotions(motions):

    for motion in motions:
        if motion == "f":
            straightMotionD(10)
        elif motion == "ql":
            circularMotion(vr=4, direction="left", R=10, angle=pi/2)
        elif motion == "qr":
            circularMotion(vr=4, direction="right", R=10, angle=pi/2)
        elif motion == "hl":
            straightMotionD(5)
            circularMotion(vr=3, direction="left", R=5, angle=pi)
            straightMotionD(5)
        elif motion == "hr":
            straightMotionD(5)
            circularMotion(vr=3, direction="right", R=5, angle=pi)
            straightMotionD(5)
        elif motion == "il":
            rotationInPlace('left', pi/2, 0.6)
        elif motion == "ir":
            rotationInPlace('right', pi/2, 0.6)


def pathPlanning(start_node, end_node):
    global motion_theta
    loadGraph()
    waypoints = bfsToList(MAZE.bfs(start_node, end_node))

    print(f'Start node:\t{start_node}')
    print(f'End node:\t{end_node}')
    print(f'BFS path: {waypoints}')
    motion_theta = firstTheta(waypoints[0], waypoints[1])
    # print(motion_theta)
    
    print(f'Rotating until {motion_theta} degrees...')
    rotateUntilAngle(motion_theta)
    motions = generateMotions(waypoints)
    # print(motions)
    runMotions(motions)
    spin()

def spin():
    while robot.step(timestep) != -1:
        setSpeedIPS(-2, 2)

# main loop
while robot.step(timestep) != -1:
    pathPlanning("3,3", "1,1")
    # pathPlanning("3,3", "1,1")
    exit()