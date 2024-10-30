import sys
import os
import numpy as np
import pinocchio

# urdf_file = 'example-robot-data/robots/ur_description/urdf/ur10_robot.urdf'
# urdf_file = 'example-robot-data/robots/talos_data/robots/talos_reduced_corrected.urdf'
# urdf_file = 'example-robot-data/robots/solo_description/robots/solo.urdf'
urdf_file = 'example-robot-data/robots/anymal_c_simple_description/urdf/anymal.urdf'
# urdf_file = 'example-robot-data/robots/panda_description/urdf/panda.urdf'
# urdf_file = 'example-robot-data/robots/double_pendulum_description/urdf/double_pendulum_simple.urdf'
# urdf_file = 'example-robot-data/robots/double_pendulum_description/urdf/double_pendulum.urdf'

# Set this to true if you want to add a Free-flyer joint a top or robot hierarchy 
# (i.e. the robot has a 6-dofs free floating base)
useFFRootJoint=False

if not "URDF_MODEL_PATH" in os.environ:
  if "ROS_PACKAGE_PATH" in os.environ:
    print('URDF_MODEL_PATH environment variable is not set. Will try to use ROS_PACKAGE_PATH to look for URDF models...')
    model_path = os.environ['ROS_PACKAGE_PATH']
  else:
    sys.exit('Please set URDF_MODEL_PATH environment variable to URDF model directory')

model_path = os.environ['URDF_MODEL_PATH']

if not os.path.isdir(model_path):
  sys.exit('Given model path does not exists')

urdf_full_filename = os.path.join(model_path, urdf_file)

if not os.path.isfile(urdf_full_filename):
  sys.exit('Given URDF file does not exists. File: ' + urdf_full_filename)

class Robot:

    def __init__(self, node):
        self.node = node

    def addRobot(self, name='RobotNode'):
        # Robot node
        robotNode = self.node.addChild(name)
        robotNode.addObject('EulerImplicitSolver')
        # robotNode.addObject('CGLinearSolver', name='Solver', iterations=200)
        robotNode.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
        robotNode.addObject('GenericConstraintCorrection')
        urdfLoader = robotNode.addObject('URDFModelLoader', name='URDFModelLoader', filename=urdf_full_filename, modelDirectory=model_path, useFreeFlyerRootJoint=useFFRootJoint, printLog=True)

        jointsNode = robotNode.getChild('Joints')
        dofs = jointsNode.getObject('dofs')
        nqWithoutRootJoint = dofs.size.value
        # Beware that configuration space number of parameters != configuration space dimension
        # and its tangent space may have a different number of parameters
        print('Robot configuration space number of parameters (without Free-flyer root joint if any) = ', nqWithoutRootJoint)

        # Add a spring force field to its rest configuration
        jointsNode.addObject('RestShapeSpringsForceField', stiffness=1e3, points=list(range(nqWithoutRootJoint)))

        # Also add a spring force field to root Free-flyer joint rest configuration, if any
        if useFFRootJoint:
          rootJointNode = robotNode.getChild('RootJoint')
          rootJointNode.addObject('RestShapeSpringsForceField', stiffness=1e3, angularStiffness=1e3, points=[0])

        return robotNode


# Test/example scene
def createScene(rootNode):

    from header import addHeader
    from robotGUI import RobotGUI

    addHeader(rootNode)

    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [TriangleCollisionModel]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm') # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection') # Needed to use components [MinProximityIntersection]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact') # Needed to use components [CollisionResponse]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshOBJLoader]

    rootNode.addObject('CollisionPipeline', verbose=False, draw=False)
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase', name="narrowPhase")
    rootNode.addObject('MinProximityIntersection', name="Proximity", alarmDistance=0.08, contactDistance=0.05, useSurfaceNormals=False)
    # rootNode.addObject('CollisionResponse', response="PenalityContactForceField")
    rootNode.addObject('CollisionResponse', response="FrictionContactConstraint")

    # Add cube for testing collisions
    cubeNode = rootNode.addChild('Cube')
    cubeMecObj = cubeNode.addObject('MechanicalObject', template="Rigid3d", translation=[0.5, 1., 0.])
    cubeNode.addObject('UniformMass', totalMass=100.0)
    visualCubeNode = cubeNode.addChild('Visual')
    visualCubeNode.addObject('MeshOBJLoader', name="cubeMeshLoader", filename="mesh/smCube27.obj", scale=0.1)
    visualCubeNode.addObject('OglModel', name="cubeVisual", src="@cubeMeshLoader", color=[0.0, 0.5, 0.5, 1.0])
    visualCubeNode.addObject('RigidMapping', input="@..", output="@cubeVisual")
    collisionCubeNode = cubeNode.addChild('Collision')
    collisionCubeNode.addObject('MeshOBJLoader', name="cubeMeshLoader", filename="mesh/smCube27.obj", scale=0.1, triangulate=True)
    collisionCubeNode.addObject('MeshTopology', src="@cubeMeshLoader")
    collisionCubeNode.addObject('MechanicalObject', src="@cubeMeshLoader")
    collisionCubeNode.addObject('RigidMapping', input="@..", output="@.")
    collisionCubeNode.addObject('TriangleCollisionModel')
    collisionCubeNode.addObject('LineCollisionModel')
    collisionCubeNode.addObject('PointCollisionModel')

    # Robot
    robot = Robot(rootNode)
    robotNode = robot.addRobot()

    # RobotGUI can be used to set interactively robot rest configuration
    robotNode.addObject(RobotGUI(robot = robotNode))

    return
