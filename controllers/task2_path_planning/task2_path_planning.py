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
mappingbool = True
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
        self.tiles = []
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
            try:
                tl = self.tiles[i][0]
                br = self.tiles[i][3]
            except:
                continue
            x, y = pose.x, pose.y

            if x > tl[0] and x < br[0]:
                if y < tl[1] and y > br[1]:
                    return i+1
        return -1
    
    def generateGrid(self):
        self.grid = []
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
            
    def wfp_bfs_queue_helper(self, paths, queue, nodes, cur_node, node_weights):
        for node in nodes:
            if node != "wall": queue.append(node)
            if node not in paths:
                node_weights[node] = len(paths[cur_node]) # wave front planner stuff
                new_path = copy.deepcopy(paths[cur_node])
                new_path.append(node)
                paths[node] = new_path

        return paths, queue 

    def wfp_bfs(self, start_node, search_node):
        visited = [] # visited and path may be the same for bfs, test later!
        paths = {start_node: [start_node]}

        # attach to path once the node is explored
        queue = [start_node]
        # use a list as a queue
        # pop top of the queue and add neighbors
        node_weights = {}
        node_weights[start_node] = 0
        while len(queue) > 0:
            cur_node = queue.pop(0)
            if cur_node not in visited:
                visited.append(cur_node)
                paths, queue = self.wfp_bfs_queue_helper(paths, queue, self.graph[cur_node], cur_node, node_weights)

        return paths[search_node], node_weights

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
        global prev_l, prev_r, mappingbool
        if mappingbool: self.printRobotPose(MAZE)
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

# return True if there is no wall in any of the 4 directions (90, 180, 0, 270)
# False if there is a wall
# basically returns the valid edges in a graph
def checkWalls(theta):
    # front, left, right, back
    lidar = getLidar()
    no_wall = []
    for lid in lidar:
        if lid < 6:
            no_wall.append(False)
        else:
            no_wall.append(True)
    
    if theta < 94 and theta > 86:
        return no_wall 
    elif theta < 184 and theta > 176:
        return [no_wall[2], no_wall[0], no_wall[3], no_wall[1]]
    elif theta < 274 and theta > 266:
        return [no_wall[3], no_wall[2], no_wall[1], no_wall[0]]
    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        return [no_wall[1], no_wall[3], no_wall[0], no_wall[2]]

# forward, left, right, backward
# check if there are walls
def neighTiles(tile, theta=90):
    grid = MAZE.grid 
    valid_neigh = []
    n = MAZE.n
    i = tile//n
    j = tile%n

    cur_node = str(i)+","+str(j)

    cur_node_neigh = []
    coma_indx = cur_node.find(',')
    node_i = int(cur_node[:coma_indx])
    node_j = int(cur_node[coma_indx+1:])
    
    cur_node_neigh.append(str(node_i-1)+","+str(node_j))
    cur_node_neigh.append(str(node_i)+","+str(node_j-1))
    cur_node_neigh.append(str(node_i)+","+str(node_j+1))
    cur_node_neigh.append(str(node_i+1)+","+str(node_j))

    #up
    if i == 0: valid_neigh.append(False)
    else:
        if grid[i-1][j] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)
    # left 
    if j == 0: valid_neigh.append(False)
    else:
        if grid[i][j-1] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)
    # right
    if j == n-1: valid_neigh.append(False)
    else:
        if grid[i][j+1] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)
    # down
    if i == n-1:valid_neigh.append(False)
    else:
        if grid[i+1][j] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)

    
    valid_walls = checkWalls(theta)
    for i in range(len(valid_walls)):
        if valid_walls[i] == False:
            cur_node_neigh[i] = "wall"
            valid_neigh[i] = False

    for neigh in cur_node_neigh:
        if neigh == "wall": continue
        if neigh not in MAZE.graph:
            MAZE.graph[neigh] = []

    MAZE.graph[cur_node] = cur_node_neigh
    return valid_neigh

stack = []
# helper, used to append for backtraking
def traversalStrightHelper():
    straightMotionD(10)
    stack.append(1)
    
# rotation_angle = pi/2
rotation_angle = 90
def traversalRotationtHelper():
    rotationInPlace('left', rotation_angle, 0.6)
    stack.append(0)

# find in which direction to rotate once a valid neighbor is found
def traversalRotationtHelper(theta, neighbors):

    if theta < 94 and theta > 86:
        if neighbors[1]:
            rotationInPlace('left', rotation_angle, 0.6)
            stack.append(0)
        elif neighbors[2]:
            rotationInPlace('right', rotation_angle, 0.6)
            stack.append(-1)
        elif neighbors[3]:
            rotationInPlace('left', rotation_angle, 0.6)
            rotationInPlace('left', rotation_angle, 0.6)
            stack.append(-1)
            stack.append(-1)

    elif theta < 184 and theta > 176:
        if neighbors[3]:
            rotationInPlace('left', rotation_angle, 0.6)
            stack.append(0)
            return
        elif neighbors[0]:
            rotationInPlace('right', rotation_angle, 0.6)
            stack.append(-1)
        elif neighbors[2]:
            rotationInPlace('left', rotation_angle, 0.6)
            rotationInPlace('left', rotation_angle, 0.6)
            stack.append(-1)
            stack.append(-1)

    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        if neighbors[0]:
            rotationInPlace('left', rotation_angle, 0.6)
            stack.append(0)
        elif neighbors[3]:
            rotationInPlace('right', rotation_angle, 0.6)
            stack.append(-1)
        elif neighbors[1]:
            rotationInPlace('righ', rotation_angle, 0.6)
            rotationInPlace('righ', rotation_angle, 0.6)
            stack.append(0)
            stack.append(0)

    elif theta < 274 and theta > 266:
        if neighbors[2]:
            rotationInPlace('left', rotation_angle, 0.6)
            stack.append(0)
        elif neighbors[1]:
            rotationInPlace('right', rotation_angle, 0.6)
            stack.append(-1)
        elif neighbors[0]:
            rotationInPlace('righ', rotation_angle, 0.6)
            rotationInPlace('righ', rotation_angle, 0.6)
            stack.append(0)
            stack.append(0)

n_i, n_j = 0, 0
def normalizeGraph(graph):
    global n_i, n_j
    normalized = {}
    min_i, min_j = 999, 999

    for node in graph:
        coma_indx = node.find(',')
        i = int(node[:coma_indx])
        j = int(node[coma_indx+1:])

        if min_i > i: min_i = i
        if min_j > j: min_j = j
    
    for node in graph:
        new_neighbors = copy.deepcopy(graph[node])

        for k in range(4):
            if new_neighbors[k] == "wall": continue
            coma_indx = new_neighbors[k].find(',')
            i = int(new_neighbors[k][:coma_indx]) - min_i
            j = int(new_neighbors[k][coma_indx+1:]) - min_j
            new_neighbors[k] = str(i) + ',' + str(j)

        coma_indx = node.find(',')
        i = int(node[:coma_indx]) - min_i
        j = int(node[coma_indx+1:]) - min_j

        normalized[str(i) + ',' + str(j)] = new_neighbors
    n_i, n_j = min_i, min_j
    return normalized

def spin():
    while robot.step(timestep) != -1:
        setSpeedIPS(-2, 2)

# DFS traversal, uses a global stack to keep backtrack
# the neighbors are found locally, and are not stored in a DS
target_visited_nodes = 16
def traverse():
    flag = False
    ones = sum([i.count(1) for i in MAZE.grid])

    if ones == target_visited_nodes: # all nodes already discovered
        flag = True

    if flag: # all nodes found
        print(f'DFS completion time: {robot.getTime():.2f}s')
        neighTiles(ROBOT_POSE.tile-1, ROBOT_POSE.theta)
        setSpeedIPS(-2, 2)

        MAZE.graph = normalizeGraph(MAZE.graph)
        MAZE.n = 4
        MAZE.generateTiles()
        MAZE.generateGrid()

        return True

    n_tiles = neighTiles(ROBOT_POSE.tile-1, ROBOT_POSE.theta)
    theta = ROBOT_POSE.theta

    # print(stack)
    # BACK TRACK
    if True not in n_tiles: 
        top = stack.pop()
        if top == 1:
            straightMotionD(-10)
        elif top == 0:
            rotationInPlace('right', rotation_angle, 0.6)
        elif top == -1:
            rotationInPlace('left', rotation_angle, 0.6)
    
    elif theta < 94 and theta > 86:
        if n_tiles[0]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
    elif theta < 184 and theta > 176:
        if n_tiles[1]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        if n_tiles[2]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
    elif theta < 274 and theta > 266:
        if n_tiles[3]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
    return False

########################## Motion logic ######################## 
# 5.024 = max speed in in per second
def straightMotionD(D):
    V=5.024
    is_neg = False
    if D < 0:
        is_neg = True
        D = abs(D)

    start_position = getPositionSensors()[0]
    # Calculates velocity of each motor and the robot
    phi = V / w_r                # rad/sec

    if is_neg:
        setSpeedIPS(-phi*w_r, -phi*w_r)
    else:
        setSpeedIPS(phi*w_r, phi*w_r)
    while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
        if w_r*abs(getPositionSensors()[0] - start_position) >= D-0.01:
            setSpeedIPS(0, 0)
            ROBOT_POSE.updatePose(MAZE)
            break

        ROBOT_POSE.updatePose(MAZE)

# assume angle is in radians
def rotationInPlace(direction, degree, v):
    # Determines Rotation and sets proper speeds
    if direction == "left":
        degree *= -1

    if degree < 0 :
        sign = -1
    else:
        sign = 1
        
    X_rad = math.radians(degree)
    phi = sign*2 # defualt 2

    # Calculates time need for rotation
    omega = 2*abs(phi)*w_r / distBtwWhe
    T = abs(X_rad / omega)
    # end_heading = (predicted_pose[3] - degree)%360

    t_start = robot.getTime()

    setSpeedIPS(phi*w_r, -phi*w_r)

    starting_theta = round(imuCleaner(imu.getRollPitchYaw()[2]))
    end_heading = round((starting_theta - degree)%360,2)

    marg_error = .01

    while robot.step(timestep) != -1:
        current_heading = imuCleaner(imu.getRollPitchYaw()[2])
        east_flag = True if end_heading <= 4 or end_heading >= 356 else False
        if (robot.getTime() - t_start) >= T:

            if east_flag:
                current_heading = current_heading - 360 if current_heading > 355 else current_heading
            if current_heading > (end_heading+marg_error):
                setSpeedIPS(.01, -.01)
            elif current_heading < (end_heading-marg_error):
                setSpeedIPS(-.01, .01)
            else:
                setSpeedIPS(0,0)
                ROBOT_POSE.updatePose(MAZE)
                break

        ROBOT_POSE.updatePose(MAZE)

def circleR(R,V=4,direction='right',percent = 1):
    # Determines direction and sets proper speeds
    axel_length = distBtwWhe
    wheel_radius = w_r
    omega = V/abs(R)
    if R < 0 :
        sign = -1
    else:
        sign = 1

    # Right and Clockwise
    if direction == 'right' and sign > 0:
        vl = omega*(abs(R) + sign*(axel_length/2))
        vr = omega*(abs(R) - sign*(axel_length/2))
        D = 2*math.pi*(abs(R) + sign*(axel_length/2))
    # Right and Counter Clockwise
    elif direction == 'right' and sign < 0:
        vl = -omega*(abs(R) - sign*(axel_length/2))
        vr = -omega*(abs(R) + sign*(axel_length/2))
        D = 2*math.pi*(abs(R) - sign*(axel_length/2))
    # Left and Clockwise
    elif direction == 'left' and sign > 0:
        vl = -omega*(abs(R) - sign*(axel_length/2))
        vr = -omega*(abs(R) + sign*(axel_length/2))
        D = 2*math.pi*(abs(R) - sign*(axel_length/2))
    # Left and Counter Clockwise
    elif direction == 'left' and sign < 0:
        vl = omega*(abs(R) + sign*(axel_length/2))
        vr = omega*(abs(R) - sign*(axel_length/2))
        D = 2*math.pi*(abs(R) + sign*(axel_length/2))

    D = D*percent
    phi_l = vl/wheel_radius
    phi_r = vr/wheel_radius

    # Checks to see if speed is to high 
    if abs(phi_l) > leftMotor.getMaxVelocity() or abs(phi_r) > rightMotor.getMaxVelocity():
        print("Speed is too great for robot")
        return

    # Gets starting postion of left wheel encoder to use as a stoping condition
    start_position = leftposition_sensor.getValue()
    # Sets motor speeds and sets start time
    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

    while robot.step(timestep) != -1:
    # Checks if wheel distance is larger than D
        if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.02:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
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
    global motion_theta
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
            if c[1] > d[1] and c[0] == d[0]:
                motion_theta = 180
                return "hl"
        else:
            if c[1] > d[1] and c[0] == d[0]:
                motion_theta = 180
                return "hr"

    elif motion_theta == 90:
        # going up
        if a[1] > c[1]:
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
            if c[1] < d[1] and c[0] == d[0]:
                motion_theta = 0
                return "hr"
        else:
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

        if b[0] > a[0]:
            motion_theta = 270
            return "ir"
        elif b[0] < a[0]:
            motion_theta = 90
            return "il"
        
    elif motion_theta == 90:
        if b[1] > a[1]:
            motion_theta = 0
            return "ir"
        elif b[1] < a[1]:
            motion_theta =180 
            return "il"

    elif motion_theta == 180:
        if b[0] > a[0]:
            motion_theta = 270
            return "il"
        elif b[0] < a[0]:
            motion_theta = 90
            return "ir"
        
    elif motion_theta == 270:
        if b[1] > a[1]:
            motion_theta = 0
            return "il"
        elif b[1] < a[1]:
            motion_theta =180 
            return "ir"
        
    return False

# f == forward 10, ql = quarter circle left, qr = quarter circle right
# hl = half circle left, hr = half circle right
# il, ir =  rotation in place left or right
def generateMotions(waypoints):
    global motion_theta
    motions = []

    for x in range(len(waypoints)):
        length = len(waypoints) 
        if length <= 0:
            break
        elif length == 1:
            waypoints.pop(0)

        elif length == 2:
            motions.append([motion_theta, "f"])
            waypoints.pop(0)
            waypoints.pop(0)

        elif length == 3:
            a = waypoints[0]
            b = waypoints[1]
            c = waypoints[2]

            is_forward = forwardHelper(a, b, c)
            if is_forward:
                waypoints.pop(0)
                motions.append([motion_theta,"f"])
                continue

            q_circle = quarterCircleHelper(a,b,c)
            if q_circle != False:
                motions.append([motion_theta,q_circle])
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
                motions.append([motion_theta,"f"])
                continue

            h_circle = halfCircleHelper(a,b,c,d)
            if h_circle == "hl" or h_circle == "hr":
                motions.append([motion_theta,h_circle])
                waypoints.pop(0)
                waypoints.pop(0)
                waypoints.pop(0)

                if len(waypoints) >= 2:
                    rotate = rotationHelper(waypoints[0], waypoints[1])
                    if rotate != False: 
                        motions.append([motion_theta, rotate])
                continue

            q_circle = quarterCircleHelper(a,b,c)
            if q_circle == "ql" or q_circle == "qr":
                motions.append([motion_theta,q_circle])
                waypoints.pop(0)
                waypoints.pop(0)

                if len(waypoints) >= 2:
                    rotate = rotationHelper(waypoints[0], waypoints[1])
                    if rotate != False: 
                        motions.append([motion_theta, rotate])
                continue
        
    return motions

def runMotions(motions):
    rotating_angle = 90
    distance = 10
    print("Running motions...")
    for m in motions:
        circleV = 2.0
        motion = m[1]
        if motion == "f":
            print("-  Forward 10in")
            straightMotionD(distance)
        elif motion == "ql":
            print("-  π/2 Left circular motion, R=10in")
            circleR(R=-distance, V=circleV, direction="left", percent=0.25)
        elif motion == "qr":
            print("-  π/2 Right circular motion, R=10in")
            circleR(R=distance, V=circleV, direction="right", percent=0.25)
        elif motion == "hl":
            print("-  Forward 5in")
            straightMotionD(distance/2)
            print("-  π Left circular motion, R=5in")
            circleR(R=-distance/2, V=circleV, direction="left", percent=0.5)
            print("-  Forward 5in")
            straightMotionD(distance/2)
        elif motion == "hr":
            print("-  Forward 5in")
            straightMotionD(distance/2)
            print("-  π Right circular motion, R=5in")
            circleR(R=distance/2, V=circleV, direction="right", percent=0.5)
            print("-  Forward 5in")
            straightMotionD(distance/2)
        elif motion == "il":
            print("-  π/2 Left Rotation-in-Place")
            rotationInPlace('left', rotating_angle, 0.6)
        elif motion == "ir":
            print("-  π/2 Right Rotation-in-Place")
            rotationInPlace('right', rotating_angle, 0.6)
        else:
            straightMotionD(0)

def spin():
    while robot.step(timestep) != -1:
        setSpeedIPS(-2, 2)


def printWFP(graph, wpfVals):

    print("Wave-Front Planner")

    n = 4
    for i in range(n):
        print("\t", end="  ") 
        for j in range(n):

            s = str(i) + ',' + str(j)
            neighbors = graph[s]

            if neighbors[0] == 'wall':
                print("─", end="  ")
            else:
                print("", end="   ")
        print("")

        for j in range(n):

            if j == 0: print("\t|", end="")

            s = str(i) + ',' + str(j)
            neighbors = graph[s]
            if neighbors[2] == 'wall':
                print(f' {wpfVals[s]}|', end="")
            else:
                print(f' {wpfVals[s]} ', end="")
                
        print("")

    print("\t  ─  ─  ─  ─")

def nodesToTiles(path):
    ret = []

    for node in path:
        coma_indx = node.find(',')
        i = int(node[:coma_indx])
        j = int(node[coma_indx+1:])

        ret.append(i*4 + j + 1)

    return ret

def normalizeRobotPose(tile):
    global n_i, n_j
    i = tile//8 - n_i
    j = (tile%8) - n_j
    return(i*4 + j + 1)


def pathPlanningRotationHelper(theta, target):
    if theta < 94 and theta > 86:
        if target == 90: 
            return
        elif target == 0: 
            rotationInPlace('right', rotation_angle, 0.6)
        elif target == 180: 
            rotationInPlace('left', rotation_angle, 0.6)
        elif target == 270: 
            rotationInPlace('left', rotation_angle, 0.6)
            rotationInPlace('left', rotation_angle, 0.6)

    elif theta < 184 and theta > 176:
        if target == 90: 
            rotationInPlace('right', rotation_angle, 0.6)
        elif target == 0: 
            rotationInPlace('left', rotation_angle, 0.6)
            rotationInPlace('left', rotation_angle, 0.6)
        elif target == 180: 
            return
        elif target == 270: 
            rotationInPlace('left', rotation_angle, 0.6)

    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        if target == 90: 
            rotationInPlace('left', rotation_angle, 0.6)
        elif target == 0: 
            return
        elif target == 180: 
            rotationInPlace('left', rotation_angle, 0.6)
            rotationInPlace('left', rotation_angle, 0.6)
        elif target == 270: 
            rotationInPlace('right', rotation_angle, 0.6)
    elif theta < 274 and theta > 266:
        if target == 90: 
            rotationInPlace('left', rotation_angle, 0.6)
            rotationInPlace('left', rotation_angle, 0.6)
        elif target == 0: 
            rotationInPlace('left', rotation_angle, 0.6)
        elif target == 180: 
            rotationInPlace('right', rotation_angle, 0.6)
        elif target == 270: 
            return

rotateUntilAngle(90)
def pathPlanning(start_node, end_node):
    global motion_theta
    startLocation = False

    if startLocation:
        loadGraph()
        path, wfp_vals = MAZE.wfp_bfs(start_node, end_node)
        printWFP(MAZE.graph, wfp_vals)
        
        waypoints = bfsToList(path)

        motion_theta = firstTheta(waypoints[0], waypoints[1])
        rotateUntilAngle(motion_theta)

        print("Wave-Front Planner path: " + str(nodesToTiles(path)))
        motion_theta = firstTheta(waypoints[0], waypoints[1])

        start_time = robot.getTime()
        motions = generateMotions(waypoints)
        
        runMotions(motions)
        print(f'Goal found in: {(robot.getTime()-start_time):.2f}s')

        print("sping :)")
        spin()
    else:
        if traverse():
            global mappingbool
            mappingbool = False
            ROBOT_POSE.tile = normalizeRobotPose(ROBOT_POSE.tile-1)
            
            top_left = MAZE.tiles[ROBOT_POSE.tile-1][0]
            ROBOT_POSE.x = top_left[0]+5
            ROBOT_POSE.y = top_left[1]-5

            MAZE.updateGrid(ROBOT_POSE.tile-1)
            # print("Map normalized: ")
            # ROBOT_POSE.printRobotPose(MAZE)

            print("Going to starting node... ")
            cur_i = (ROBOT_POSE.tile-1)//4
            cur_j = (ROBOT_POSE.tile-1)%4

            cur_node = str(cur_i)+","+str(cur_j)

            path, wfp_vals = MAZE.wfp_bfs(cur_node, start_node)
            printWFP(MAZE.graph, wfp_vals)

            waypoints = bfsToList(path)

            motion_theta = firstTheta(waypoints[0], waypoints[1])
            pathPlanningRotationHelper(ROBOT_POSE.theta, motion_theta)

            print("Wave-Front Planner path: " + str(nodesToTiles(path)))
            motion_theta = firstTheta(waypoints[0], waypoints[1])

            start_time = robot.getTime()
            motions = generateMotions(waypoints)
            
            runMotions(motions)
            print(f'Initial node found in: {(robot.getTime()-start_time):.2f}s')

            MAZE.generateGrid()
            MAZE.updateGrid(ROBOT_POSE.tile-1)
            coma_indx = cur_node.find(',')
            node_i = int(cur_node[:coma_indx])
            node_j = int(cur_node[coma_indx+1:])
            ROBOT_POSE.tile = node_i*4 + node_j + 1
            MAZE.updateGrid(ROBOT_POSE.tile-1)
            ROBOT_POSE.theta = imuCleaner(imu.getRollPitchYaw()[2])


            path, wfp_vals = MAZE.wfp_bfs(start_node, end_node)
            printWFP(MAZE.graph, wfp_vals)

            waypoints = bfsToList(path)

            motion_theta = firstTheta(waypoints[0], waypoints[1])
            pathPlanningRotationHelper(ROBOT_POSE.theta, motion_theta)

            print("Wave-Front Planner path: " + str(nodesToTiles(path)))
            motion_theta = firstTheta(waypoints[0], waypoints[1])

            start_time = robot.getTime()
            motions = generateMotions(waypoints)

            runMotions(motions)
            print(f'Goal found in: {(robot.getTime()-start_time):.2f}s')

            print("sping :)")

            spin()

# main loop
while robot.step(timestep) != -1:
    pathPlanning("0,0", "3,2")
