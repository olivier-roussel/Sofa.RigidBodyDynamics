import sys
import os
import numpy as np
import pinocchio

# urdf_file = 'example-robot-data/robots/ur_description/urdf/ur10_robot.urdf'
# urdf_file = 'example-robot-data/robots/talos_data/robots/talos_reduced_corrected.urdf'
# urdf_file = 'example-robot-data/robots/solo_description/robots/solo.urdf'
# urdf_file = 'example-robot-data/robots/panda_description/urdf/panda.urdf'
urdf_file = 'example-robot-data/robots/double_pendulum_description/urdf/double_pendulum_simple.urdf'
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
        self.node.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [TriangleCollisionModel]  
        self.node.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
        self.node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
        self.node.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]

        # Robot node
        robotNode = self.node.addChild(name)
        robotNode.addObject('EulerImplicitSolver')
        #robotNode.addObject('CGLinearSolver', name='Solver', iterations=200)
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

    # Robot
    robot = Robot(rootNode)
    robotNode = robot.addRobot()

    # RobotGUI can be used to set interactively robot rest configuration
    robotNode.addObject(RobotGUI(robot = robotNode))

    return
