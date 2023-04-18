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
            # motion_theta = 270
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
        # print("--------------------------------------------------")
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
                # print(f'points: {a}, {b}, {c}, {d}')
                # print(f'cur: {motion_theta}, motion: f')

                waypoints.pop(0)
                motions.append([motion_theta,"f"])
                continue

            h_circle = halfCircleHelper(a,b,c,d)
            if h_circle == "hl" or h_circle == "hr":
                # print(f'points: {a}, {b}, {c}, {d}')
                # print(f'prev: cur: {motion_theta}, motion: {h_circle}')

                motions.append([motion_theta,h_circle])
                waypoints.pop(0)
                waypoints.pop(0)
                waypoints.pop(0)

                if len(waypoints) >= 2:
                    rotate = rotationHelper(waypoints[0], waypoints[1])
                    if rotate != False: 
                        motions.append([motion_theta, rotate])
                        # print(f'cur: {motion_theta}, motion: {rotate}')
                continue

            q_circle = quarterCircleHelper(a,b,c)
            if q_circle == "ql" or q_circle == "qr":
                # print(f'points: {a}, {b}, {c}, {d}')
                # print(f'cur: {motion_theta}, motion: {q_circle}')
                motions.append([motion_theta,q_circle])
                waypoints.pop(0)
                waypoints.pop(0)

                if len(waypoints) >= 2:
                    # print(f'points: {waypoints[0]}, { waypoints[1]}')
                    rotate = rotationHelper(waypoints[0], waypoints[1])
                    if rotate != False: 
                        motions.append([motion_theta, rotate])
                        # print(f'cur: {motion_theta}, motion: {rotate}')
                continue
        
    return motions

# f == forward 10, ql = quarter circle left, qr = quarter circle right
# hl = half circle left, hr = half circle right
# il, ir =  rotation in place left or right 
def runMotions(motions):

    for m in motions:
        motion = m[1]
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

def spin():
    while robot.step(timestep) != -1:
        setSpeedIPS(-2, 2)


def pathPlanning(start_node, end_node):
    global motion_theta
    loadGraph()
    waypoints = bfsToList(MAZE.bfs(start_node, end_node))

    print(f'Start node: {start_node}\t End node: {end_node}')
    print(f'BFS path: {waypoints}')
    motion_theta = firstTheta(waypoints[0], waypoints[1])
    # print(motion_theta)
    
    print(f'Rotating until {motion_theta} degrees...')
    rotateUntilAngle(motion_theta)
    start_time = robot.getTime()
    motions = generateMotions(waypoints)
    print(motions)
    runMotions(motions)
    print(f'Goal found in: {(robot.getTime()-start_time):.2f}s')
    spin()


# main loop
while robot.step(timestep) != -1:
    pathPlanning("3,3", "2,1")
    # pathPlanning("3,3", "1,1")
    exit()