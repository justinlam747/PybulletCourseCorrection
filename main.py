import pybullet as p
import pybullet_data
import time

# PyBullet Lane Corridor Follower 
# 10/02/25
# A script that course corrects a box within walls

# Connect to graphics
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF

# set gravity, in ms^2
p.setGravity(0,0,-10)

# loads a plane
planeId = p.loadURDF("plane.urdf")

# defines start position and orientation
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])


# load the robot 
boxId = p.loadURDF("cube.urdf",cubeStartPos, cubeStartOrientation)

# create a wall object
wall = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 100, 0.5])

# spawn two walls
wall1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall, basePosition=[2, 0, 0.5])
wall2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall, basePosition=[-2, 0, 0.5])

# init velocity of box
p.resetBaseVelocity(boxId, linearVelocity=[0,30,0], angularVelocity=[0,0,0])

# run sumulation for 5 seconds
for i in range (5 * 240):
    p.stepSimulation()
    time.sleep(1./240.)
    # print cube position and orientation
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos)

    # let the camera follow the box
    p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-50, cameraTargetPosition=cubePos)

p.disconnect()