import sys
import os

pinocchio_model_path='/home/olrousse/devel/mambaforge/envs/sofa-pinocchio-dev/src/pinocchio/models/'
urdf_file='example-robot-data/robots/ur_description/urdf/ur10_robot.urdf'
# urdf_file='example-robot-data/robots/bolt_description/robots/bolt.urdf'

if not os.path.isdir(pinocchio_model_path):
  sys.exit('Given model path does not exists')

if not os.path.isfile(os.path.join(pinocchio_model_path, urdf_file)):
  sys.exit('Given URDF file does not exists')

class Robot:

    def __init__(self, node):
        self.node=node

    def addRobot(self, name='Robot', translation=[0,0,0]):

        initAngles = [0,0,0,0,0,0,0] # 7 angles

        # Robot node
        robot = self.node.addChild(name)
        # robot.addData('angles', initAngles, None, 'angle of articulations in radian', '', 'vector<float>')
        robot.addData('angles', initAngles, None, 'angle of articulations in radian', '', 'vector<float>')
        robot.addObject('EulerImplicitSolver')
        robot.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
        robot.addObject('GenericConstraintCorrection')
        # TODO add way to add reference configuration in the URDFModelLoader, e.g. with
        # q_rest = [0,0,0,0,0,0]
        robot.addObject('URDFModelLoader', name='URDFModelLoader', urdfFilename=os.path.join(pinocchio_model_path, urdf_file), modelDirectory=pinocchio_model_path, printLog=True)

        return robot


# Test/example scene
def createScene(rootNode):

    from header import addHeader
    from robotGUI import RobotGUI  # Uncomment this if you want to use the GUI

    addHeader(rootNode)

    # Robot
    robot = Robot(rootNode).addRobot()
    robot.addObject(RobotGUI(robot=robot))  # Uncomment this if you want to use the GUI

    return
