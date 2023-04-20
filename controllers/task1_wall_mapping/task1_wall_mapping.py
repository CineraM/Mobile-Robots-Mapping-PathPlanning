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

    # code that generates the 4 points for all tiles, n*n tiles
    # generates top left, top right, bottom left, bottom right points of an nxn grid
    def generateTiles(self, top_left=20, step=10):
        y = top_left
        for i in range(self.n):
            x = -top_left
            for j in range(self.n):
                self.tiles.append([[x, y], [x+step, y], [x, y-step], [x+step, y-step]])
                x+=step
            y-=step

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
            break

        ROBOT_POSE.updatePose(MAZE)

def rotationInPlace(direction, degree, v):
    # Determines Rotation and sets proper speeds
    if direction == "left":
        degree *= -1

    if degree < 0 :
        sign = -1
    else:
        sign = 1
        
    X_rad = math.radians(degree)
    phi = sign*2

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
                break

        ROBOT_POSE.updatePose(MAZE)
########################## Motion logic ######################## 



########################## traversal logic ######################## 
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
    #debug
    # print(valid_walls)
    # print("-------------------------")
    # print(valid_neigh)
    # print(".")
    # print(".")

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

def normalizeGraph(graph):
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
        
        for i in range(4):
            for j in range(4):
                s = str(i) + ',' + str(j)
                if s in MAZE.graph:
                    print(f'node: {s} , edges: {MAZE.graph[s]}')
        
        path = os.getcwd()
        path = os.path.dirname(path) + "/graph.json" 
        print("Saving graph to: ", path)
        with open(path, "w") as fp:
            json.dump(MAZE.graph, fp)

        spin()

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

print("Rotating until 90 degrees...")
rotateUntilAngle(90)
while robot.step(timestep) != -1:
    traverse()