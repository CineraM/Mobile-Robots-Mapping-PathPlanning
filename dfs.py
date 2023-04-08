# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math
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

# global variables
distBtwWhe = 2.28
dmid = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2
pi = math.pi
half_of_robot = 0.037*39.3701 
toIn = 39.3701


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

# code that generates the 4 points for all tiles, 16 tiles
# generates top left, top right, bottom left, bottom right points of an nxn grid
def generateTiles(n):
    y = 20
    tiles = []
    for i in range(n):
        x = -20
        for j in range(n):
            tiles.append([[x, y], [x+10, y], [x, y-10], [x+10, y-10]])
            x+=10
        y-=10
    return tiles
################# END  #################

# generates an nxn grid of 0s
def generateGrid(n):
    ret = []
    for i in range(n):
        temp = [] 
        for j in range(n):
            temp.append(0)
        ret.append(temp)
    return ret

grid = generateGrid(4)

################# Updating-Pose Code #################
# change 
def updateGrid(tile):
    global grid
    i = tile//4
    j = tile%4
    grid[i][j] = 1

tiles_coordinates = generateTiles(4)

################ robot class & functions #####################
class RobotPose:
  def __init__(self, x, y, tile, theta):
    self.x = x
    self.y = y
    self.tile = tile
    self.theta = theta 

robot_grid = generateGrid(4)
def updateRobotGrid(tile):
    global robot_grid
    i = tile//4
    j = tile%4
    grid[i][j] = 1

# bottom left, top right, robot
def findCurTile(pose):
    global tiles_coordinates
    # up, down, left, right instead looking though all the tiles
    # the search space is extremly small, this will not affect performance
    for i in range(len(tiles_coordinates)):
        tl = tiles_coordinates[i][0]
        br = tiles_coordinates[i][3]
        x, y = pose.x, pose.y

        if x > tl[0] and x < br[0]:
            if y < tl[1] and y > br[1]:
                return i+1
    return -1

#print the grid & robot pose
def printRobotPose(obj):
    global grid
    cur_tile = findCurTile(obj)-1
    # i = cur_tile//4
    # j = cur_tile%4
    # if robot_grid[i][j] == 1:
    #     return
    # else:
    #     robot_grid[i][j] = 1
    print(f'x: {obj.x:.2f}\ty: {obj.y:.2f}\ttile: {obj.tile}\ttheta: {obj.theta:.2f}')
    for list in grid:
        print("\t" + str(list))
    print("-----------------------------------------------")

ROBOT_POSE = RobotPose(15.0, -15.0, 16, 90)
updateGrid(ROBOT_POSE.tile-1)
prev_l, prev_r = getPositionSensors()

# bottom left, top right, robot
def updateTile(pose):
    global tiles_coordinates
    # up, down, left, right instead looking though all the tiles
    # the search space is extremly small, this will not affect performance
    for i in range(len(tiles_coordinates)):
        tl = tiles_coordinates[i][0]
        br = tiles_coordinates[i][3]
        x, y = pose.x, pose.y

        if x > tl[0] and x < br[0]:
            if y < tl[1] and y > br[1]:
                return i+1
    return -1

# Update pose of robot and grid, updates if a tile is found
def updatePose(obj):
    global prev_l, prev_r, ROBOT_POSE
    printRobotPose(ROBOT_POSE)
    cur_l, cur_r = getPositionSensors()
    vl = (cur_l-prev_l)/0.032   # 32 ms 
    vr = (cur_r-prev_r)/0.032
    imu_reading = imuCleaner(imu.getRollPitchYaw()[2])
    dist = distAfterTask(vl*w_r, vr*w_r)
    obj.theta = imu_reading
    
    prev_l = cur_l
    prev_r = cur_r

    if imu_reading < 94 and imu_reading > 86:
        obj.y += dist
    elif imu_reading < 184 and imu_reading > 176:
        obj.x -= dist
    elif imu_reading < 274 and imu_reading > 266:
        obj.y -= dist
    elif imu_reading <= 360 and imu_reading > 356 or imu_reading < 4 and imu_reading >= 0:
        obj.x += dist

    tile = updateTile(obj)
    if tile != -1: 
        obj.tile = tile
        updateGrid(tile-1)
################ robot class & functions #####################



################ motion functions #####################
# 5.024 = max speed in in per second
def straightMotionD(d):
    global ROBOT_POSE
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
            updatePose(ROBOT_POSE)
            printRobotPose(ROBOT_POSE)
            break
        if is_neg:
            setSpeedIPS(-v, -v)
        else:
            setSpeedIPS(v, v)
        updatePose(ROBOT_POSE)
        printRobotPose(ROBOT_POSE)

# assume angle is in radians
def rotationInPlace(direction, angle, v):
    global ROBOT_POSE
    s = angle*dmid
    time = s/v
    s_time = robot.getTime()

    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            updatePose(ROBOT_POSE)
            printRobotPose(ROBOT_POSE)
            break 
        if direction == "left":
            setSpeedIPS(-v, v)
        else:
            setSpeedIPS(v, -v)
            
        updatePose(ROBOT_POSE)
        printRobotPose(ROBOT_POSE)
################ motion functions #####################

############## traversal logic ############
# return True if there is no wall in any of the 4 directions (90, 180, 0, 270)
# False if there is a wall
# basically returns the valid edges in a graph
def checkWalls(theta):
    # front, left, right, back
    lidar = getLidar()
    no_wall = []
    for lid in lidar:
        if lid < 4:
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
    valid_neigh = []
    n = 4 # could use as an input, not for this lab
    i = tile//n
    j = tile%n

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
    if j == n-1:valid_neigh.append(False)
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
    
    # print(valid_neigh)
    valid_walls = checkWalls(theta)
    for i in range(len(valid_walls)):
        if valid_walls[i] == False:
            valid_neigh[i] = False
        
    # print(valid_walls)
    # print(valid_neigh)
    # print("-------------------------")

    return valid_neigh

stack = []
# helper, used to append for backtraking
def traversalStrightHelper():
    global stack
    straightMotionD(10)
    stack.append(1)
    
def traversalRotationtHelper():
    global stack
    rotationInPlace('left', pi/2, 0.6)
    stack.append(0)

# find in which direction to rotate once a valid neighbor is found
def traversalRotationtHelper(theta, neighbors):

    if theta < 94 and theta > 86:
        if neighbors[1]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
        elif neighbors[2]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[3]:
            rotationInPlace('left', pi/2, 0.6)
            rotationInPlace('left', pi/2, 0.6)
            stack.append(-1)
            stack.append(-1)

    elif theta < 184 and theta > 176:
        if neighbors[3]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
            return
        elif neighbors[0]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[2]:
            rotationInPlace('left', pi/2, 0.6)
            rotationInPlace('left', pi/2, 0.6)
            stack.append(-1)
            stack.append(-1)

    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        if neighbors[0]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
        elif neighbors[3]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[1]:
            rotationInPlace('righ', pi/2, 0.6)
            rotationInPlace('righ', pi/2, 0.6)
            stack.append(0)
            stack.append(0)

    elif theta < 274 and theta > 266:
        if neighbors[2]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
        elif neighbors[1]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[0]:
            rotationInPlace('righ', pi/2, 0.6)
            rotationInPlace('righ', pi/2, 0.6)
            stack.append(0)
            stack.append(0)

# DFS traversal, uses a global stack to keep backtrack
# the neighbors are found locally, and are not stored in a DS
def traverse():
    global grid, stack, ROBOT_POSE
    flag = False
    for list in grid:
        if 0 in list: 
            flag = True
            break
    if flag == False: # victory spin :) 
        setSpeedIPS(-2, 2)
        return

    n_tiles = neighTiles(ROBOT_POSE.tile-1, ROBOT_POSE.theta)
    theta = ROBOT_POSE.theta

    # print(stack)
    # BACK TRACK
    if True not in n_tiles: 
        top = stack.pop()
        if top == 1:
            straightMotionD(-10)
        elif top == 0:
            rotationInPlace('right', pi/2, 0.6)
        elif top == -1:
            rotationInPlace('left', pi/2, 0.6)
    
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
############## traversal logic ############


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