import pybullet as p
import pybullet_data
import time, math, random

# PyBullet Lane Corridor Follower 
# 10/02/25
# A Proportional-Integral-Derivative (PID) controller implmented to keep a box aligned with the centerline of a corridor
# This script details a simple PID controller that adjusts the torque rotation based on a output factor of the PID to acheieve critcally damped movement
# The box is randomly oriented every 5 seconds proving that the PID controller can correct the heading fo the box no matter what forward angle is appleid

# connect to graphics
physicsClient = p.connect(p.DIRECT) # use p.GUI for graphical version, p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

# create wall properties
randomShards = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5,0.1,0.5])
cooridor = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15,5,0.5])

# spawn walls 
wallCount = 30
wallList = []
coorList = []
wallOrn = []

# create walls for a cooridor
for n in range(wallCount):

    # # math uses radians not degrees so convert
    # theta = math.radians(180 * random.random())
    # theta2 = math.radians(90 * random.random())

    # # half angle formula for a quaternion representation of a rotation, since we are moving in the y axis we only need to rotate around the z axis
    # q1 = [0,0,math.sin(theta/2), math.cos(theta/2)]
    # q2 = [0,0,math.sin(theta/2) - math.sin(theta2/2), math.cos(theta/2)]

    # create coordoor walls
    leftCoor = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cooridor, basePosition=[-3,n*10,0.5])
    rightCoor = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cooridor, basePosition=[3,n*10,0.5])
    coorList.extend([leftCoor, rightCoor])

    # # add the walls to the wall list
    # leftWall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=randomShards, basePosition=[-0.9,n*3,0.5], baseOrientation=q1)
    # rightWall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=randomShards, basePosition=[0.7,n*3,0.5], baseOrientation=q2)
    # wallList.extend([leftWall, rightWall])


print("Walls Created:", wallList)

p.saveBullet("cooridorBullet.bullet")
p.disconnect()

p.connect(p.GUI)
p.loadBullet("cooridorBullet.bullet")

# set gravity, in ms^2
p.setGravity(0,0,-10)

# loads a plane
planeId = p.loadURDF("plane.urdf")

# defines start position and orientation
cubeStartPos = [0,0,0.3]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
speed = 30

# load the robot 
boxId = p.loadURDF("cube.urdf",cubeStartPos, cubeStartOrientation, globalScaling=0.6)
p.changeDynamics(boxId, -1, mass=1, angularDamping=0.9, linearDamping=0.1)

# init velocity of box
p.resetBaseVelocity(boxId, linearVelocity=[0,30,0], angularVelocity=[0,0,0])
velocity, angularVelocity = p.getBaseVelocity(boxId)

line_id = p.addUserDebugLine([0,0,0], [0,5,0], [0,1,0], lineWidth=2, lifeTime=0)
travelLine_id = p.addUserDebugLine([0,0,0], [0,5,0], [1,0,0], lineWidth=2, lifeTime=0)

# assign values for pid
prev_error_x = 0
Kp = 30
Ki = 15
Kd = 3
integral = 0

# run sumulation for 5 seconds
for i in range (5 * 240):
    currentAngle = math.degrees(p.getEulerFromQuaternion(curOrn)[2])
    
    # step the simulation and sleep it to match real life
    p.stepSimulation()
    time.sleep(1./240.)

    # get position and velocity
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    velocity, angularVelocity = p.getBaseVelocity(boxId)

    # random angle between 45 and 135 degrees 
    # angle = math.pi/4 + (math.pi/2 * random.uniform(0,1))
    angle = 3* math.pi/2




    # # extremes
    # angle = math.pi/4
    # angle = 3 * math.pi/4
   
    # Every 240 steps is around a second so every 24 steps is 0.1 seconds, due to steps being frequency based and not time based it is slightly off
    # I discovered 24 steps was around 1 second due to time.sleep called earlier, leading to a nice consistent reporting speed of "car" data
    if (i%240==0): 
        print("Cube Position: ", cubePos)
        print("Cube Orientation: ", cubeOrn)
        print("Velocity:", velocity, "Angular Velocity:", angularVelocity )
        print("Randomizing Heading")

        # randomize heading
        p.resetBasePositionAndOrientation(boxId, cubePos, p.getQuaternionFromEuler([0,0,angle]))

    if (currentAngle < 45):
        print("Current Angle:", currentAngle, " - Too Far Right")
        

    # compute pid controller variables   
    error_x = cubePos[0]
    integral += error_x * (1./240.)
    derivative = (error_x - prev_error_x) / (1./240.)

    # define the factor for torque
    u = Kp * error_x + Ki * integral + Kd * derivative 

    # # basic bang-bang logic 
    # if (cubePos[0] > 0):
    #     p.applyExternalTorque(boxId, -1, [0,0,u], p.WORLD_FRAME)
        
    # if (cubePos[0] < 0):
    #     p.applyExternalTorque(boxId, -1, [0,0,-u], p.WORLD_FRAME)

    # apply the torque based on pid-controller output 
    p.applyExternalTorque(boxId, -1, [0,0,u], p.WORLD_FRAME)
       
    # get the current position and orientation of the box
    curPos, curOrn =p.getBasePositionAndOrientation(boxId)

    # rotate about z axis so 2nd index 
    lookAngle = p.getEulerFromQuaternion(curOrn)[2]
   
   # set the velocity based on where the box is facing 
    vx = speed * math.cos(lookAngle) 
    vy = speed * math.sin(lookAngle)
    p.resetBaseVelocity(boxId, linearVelocity=[vx,vy,0], angularVelocity=[0,0,0])

    # let the camera follow the box
    p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-50, cameraTargetPosition=cubePos)

    # debug line indicating travel direction 
    travelLine = p.addUserDebugLine(cubePos, [cubePos[0] + velocity[0], cubePos[1] + velocity[1], cubePos[2] + 0.1], [0,1,0], 2, 0, replaceItemUniqueId=travelLine_id)

    # draw a line in front of the box to show correct direction of travel
    centerline = p.addUserDebugLine([0,0,0.1], [0,cubePos[1] + 30, 0.1], [1,0,0], 2, 0, replaceItemUniqueId=line_id)

    # update previous error
    prev_error_x = error_x

    print()

    

p.disconnect()