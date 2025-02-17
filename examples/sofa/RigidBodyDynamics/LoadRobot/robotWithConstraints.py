import sys
import os
import numpy as np
import pinocchio
import Sofa

urdf_file = 'example-robot-data/robots/panda_description/urdf/panda.urdf'

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
        self.node.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Model') # Needed to use components [BilateralLagrangianConstraint]  
        self.node.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
        self.node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
        self.node.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]  

        # Robot node
        robotWrapperNode = self.node.addChild(name)
        robotWrapperNode.addObject('EulerImplicitSolver')
        robotWrapperNode.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
        robotWrapperNode.addObject('GenericConstraintCorrection')
        urdfLoader = robotWrapperNode.addObject('URDFModelLoader', name='URDFModelLoader', filename=urdf_full_filename, 
          modelDirectory=model_path, useFreeFlyerRootJoint=useFFRootJoint, printLog=True)

        robotNode = robotWrapperNode.getChild('Robot')
        dofs = robotNode.getObject('dofs')
        nqWithoutRootJoint = dofs.size.value
        # Beware that configuration space number of parameters != configuration space dimension
        # and its tangent space may have a different number of parameters
        print('Robot configuration space number of parameters (without Free-flyer root joint if any) = ', nqWithoutRootJoint)

        # Add a spring force field to its rest configuration
        robotNode.addObject('RestShapeSpringsForceField', stiffness=1e3, points=list(range(nqWithoutRootJoint)))

        # Also add a spring force field to root Free-flyer joint rest configuration, if any
        if useFFRootJoint:
          rootJointNode = robotWrapperNode.getChild('RootJoint')
          rootJointNode.addObject('RestShapeSpringsForceField', stiffness=1e3, angularStiffness=1e3, points=[0])

        # Add dummy mechanical object to which we will attach the robot gripper
        dummyNode = self.node.addChild('dummyNode')
        dummyNode.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
        dummyNode.addObject('EulerImplicitSolver')
        dummyNode.addObject('GenericConstraintCorrection')
        dummyNode.addObject('MechanicalObject', name="fixedFrame", template="Vec3d", position=[0.5, 0., 0.9])
        dummyNode.addObject('RestShapeSpringsForceField', stiffness=1e5, points=[0])
        dummyNode.addObject('UniformMass', totalMass=1)

        # Create constraint between gripper and the dummy mechanical object (3d position)
        constraintBodyIndex = 7
        robotNode = robotWrapperNode.getChild('Robot')
        constraintsNode = robotNode.addChild('Constraints')
        constraintsNode.addObject('MechanicalObject', name="attachedToRobotObject", template="Vec3d", position=[0., 0., 0.])
        constraintsNode.addObject('BilateralLagrangianConstraint', template="Vec3", object1="@attachedToRobotObject", object2="@dummyNode/fixedFrame", first_point="0", second_point="0")
        constraintsNode.addObject('RigidMapping', template='Rigid,Vec3d', name='robotBodyMapping', index=constraintBodyIndex, input="@Joints/jointsDof", output="@attachedToRobotObject")

        return robotWrapperNode


# Test/example scene
def createScene(rootNode):

    from header import addHeader
    from robotGUI import RobotGUI

    addHeader(rootNode)

    # Robot
    robot = Robot(rootNode)
    robotWrapperNode = robot.addRobot()

    # RobotGUI can be used to set interactively robot rest configuration
    robotWrapperNode.addObject(RobotGUI(robot = robotWrapperNode))

    return
