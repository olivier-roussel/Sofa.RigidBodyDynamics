import sys
import os
import numpy as np
import pinocchio

# urdf_file = 'example-robot-data/robots/ur_description/urdf/ur10_robot.urdf'
# urdf_file = 'example-robot-data/robots/talos_data/robots/talos_reduced_corrected.urdf'
# urdf_file = 'example-robot-data/robots/solo_description/robots/solo.urdf'
urdf_file = 'example-robot-data/robots/panda_description/urdf/panda.urdf'

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
        #robotNode.addObject('CGLinearSolver', name='Solver', iterations=200)
        robotNode.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
        robotNode.addObject('GenericConstraintCorrection')
        urdfLoader = robotNode.addObject('URDFModelLoader', name='URDFModelLoader', filename=urdf_full_filename, modelDirectory=model_path, useFreeFlyerRootJoint=useFFRootJoint, printLog=True)

        return robotNode


# Test/example scene
def createScene(rootNode):

    from header import addHeader
    from robotGUI import RobotGUI  # Uncomment this if you want to use the GUI

    addHeader(rootNode)

    # Robot
    robot = Robot(rootNode)
    robotNode = robot.addRobot()

    robotNode.addObject(RobotGUI(robot = robotNode))  # Uncomment this if you want to use the GUI

    return
