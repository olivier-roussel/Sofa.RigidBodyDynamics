import sys
import os
import numpy as np
import pinocchio

pinocchio_model_path = '/home/olrousse/devel/mambaforge/envs/sofa-pinocchio-dev/src/pinocchio/models/'
#urdf_file = 'example-robot-data/robots/ur_description/urdf/ur10_robot.urdf'
urdf_file = 'example-robot-data/robots/double_pendulum_description/urdf/double_pendulum_simple.urdf'

urdf_full_filename = os.path.join(pinocchio_model_path, urdf_file)

if not os.path.isdir(pinocchio_model_path):
  sys.exit('Given model path does not exists')

if not os.path.isfile(urdf_full_filename):
  sys.exit('Given URDF file does not exists. File: ' + urdf_full_filename)

class Robot:

    def __init__(self, node):
        self.node = node
        self.nq = 0

    def addRobot(self, name='Robot'):

        # Load the URDF model with pinocchio, only to get model num dofs for GUI elements
        # Actual URDF loading is done through the URDFModelLoader component added to the scene 
        # in the following lines
        model = pinocchio.buildModelFromUrdf(urdf_full_filename)
        self.nq = model.nq

        # reference config
        self.q0 = np.zeros(self.nq)

        # Robot node
        robotNode = self.node.addChild(name)
        robotNode.addData('q0', self.q0, None, 'angle of articulations in radian', '', 'vector<float>')
        robotNode.addObject('EulerImplicitSolver')
        robotNode.addObject('CGLinearSolver', name='Solver', iterations=200)
        # robotNode.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
        # robotNode.addObject('GenericConstraintCorrection')
        robotNode.addObject('URDFModelLoader', name='URDFModelLoader', urdfFilename=urdf_full_filename, modelDirectory=pinocchio_model_path, printLog=True)

        return robotNode


# Test/example scene
def createScene(rootNode):

    from header import addHeader
    from robotGUI import RobotGUI  # Uncomment this if you want to use the GUI

    addHeader(rootNode)

    # Robot
    robot = Robot(rootNode)
    robotNode = robot.addRobot()
    robotNode.addObject(RobotGUI(robot = robotNode, q0 = robot.q0))  # Uncomment this if you want to use the GUI

    return
