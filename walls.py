import pybullet as p
import pybullet_data


def createWalls(wallCount):
    # If I had time I was going to spawn random shards and update the navigation path using ray casting 
    # The idea was creating a forward fan of rays and checking if any rays hit a wall, if they did I would
    # take the farthest left -1 and farthest right -1 (returns -1 if no hit), take the average length distance
    # then set the heading towards that vector

    # connect to graphics
    physicsClient = p.connect(p.DIRECT) # use p.GUI for graphical version, p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

    # create wall properties
    randomShards = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5,0.1,0.5])
    cooridor = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15,5,0.5])

    # spawn walls 
    
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


def loadEnv():
   

   # By saving objects to a .bullet you can load env's instantly, instead of incrementing and delaying simulation
   p.loadBullet("cooridorBullet.bullet")

   # loads a plane
   planeId = p.loadURDF("plane.urdf",  globalScaling=6, basePosition=[0,0,0.1])
