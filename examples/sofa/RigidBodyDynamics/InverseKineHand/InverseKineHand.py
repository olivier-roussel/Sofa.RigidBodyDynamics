import os
import sys
import math
import pinocchio as pin
import numpy as np

urdf_model_path = os.path.dirname(os.path.abspath(__file__)) + '/../robots_description/schunk_hand/'
cur_path = os.path.dirname(os.path.abspath(__file__)) + '/'
urdf_file = 'schunk_svh_hand_right.urdf'
urdf_full_filename = os.path.join(urdf_model_path, urdf_file)

# initial hand configuration where it grasps the deformable object using three fingers (index, pinky and thumb)

q_init = [1.77736181, 0.09626184, 0.71877442, 0.43485226, 0.55710469, 0.40919228,
          0.93233217, 0.43232146, 0.,         0.,         0.,         0.10067708,
          0.02485863, 0.62568506, 0.29532192, 0.11736043, 0.,         0.,
          0.,         0.        ]


q_rest = [0] * len(q_init)

# scale = 100 : initial scene description in cm
scale = 1

def tfCoords(c):
  return np.array([c[0], c[1], c[2]])

goal_positions = np.array([tfCoords([0.06, 0.077, 0.12]),
                           tfCoords([0.06, 0.017, 0.177]),
                           tfCoords([0.06, -0.053, 0.12]),
                           tfCoords([0.06, 0.017, 0.12])]) * scale

effector_positions = np.array([tfCoords([0.05, 0.077, 0.12]),
                               tfCoords([0.08, 0.017, 0.157]),
                               tfCoords([0.05, -0.053, 0.12]),
                               tfCoords([0.05, 0.017, 0.12])]) * scale

finger_thumb_tip_frame_name = 'thtip'
finger_index_tip_frame_name = 'fftip'
finger_pinky_tip_frame_name = 'lftip'

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop') # Needed to use components [FreeMotionAnimationLoop]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm') # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection') # Needed to use components [LocalMinDistance]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact') # Needed to use components [RuleBasedContactManager]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglSceneFrame]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [LinearSolverConstraintCorrection]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshVTKLoader]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [TetrahedronFEMForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  

    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName=['Sofa.RigidBodyDynamics', 'SofaPython3', 'SoftRobots', 'SoftRobots.Inverse'])

    # rootNode.findData('gravity').value = [0, 0, 0]
    rootNode.findData('gravity').value = [9.81 * scale, 0, 0]
    # rootNode.findData('gravity').value = [0, 9.81 * scale, 0]
    rootNode.findData('dt').value = 0.01

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog=True, epsilon=1e-3, allowSliding=False,
                       maxIterations=5000, tolerance=1e-10, responseFriction=3., minContactForces=0.02 * scale)

    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('RuleBasedContactManager', responseParams='mu=3.', response='FrictionContactConstraint')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('LocalMinDistance', alarmDistance=0.006 * scale, contactDistance=0.001 * scale)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    ##########################################
    # Effector goal for interactive control  #
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=0.0001 * scale, threshold=0.0001 * scale)
    goal.addObject('MechanicalObject', name='goalMO', showObject=True, showObjectScale=0.003 * scale, drawMode=1,
                   showColor=[255, 255, 255, 255],
                   position = goal_positions)
    goal.addObject('SphereCollisionModel', radius=0.008 * scale, group=[1, 2])
    goal.addObject('UncoupledConstraintCorrection')

    ##########################################
    # FEM Model                              #
    ##########################################
    modelTranslation = tfCoords([0.014 * scale, 0.01 * scale, 0.014 * scale])
    modelRotation = pin.rpy.matrixToRpy(pin.utils.rotate('z', math.pi * 0.5) @ pin.utils.rotate('x', math.pi*0.5))
    # modelRotation = pin.rpy.matrixToRpy(pin.utils.rotate('z', math.pi) @ pin.utils.rotate('x', math.pi*0.5))
    modelRotationDegrees = [i * 180. / math.pi for i in modelRotation]
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver')
    model.addObject('SparseLDLSolver', template="CompressedRowSparseMatrix")
    model.addObject('MeshVTKLoader', name='loader', filename=cur_path+'bar.vtk', translation=modelTranslation, rotation=modelRotationDegrees, scale=0.001 * scale)
    model.addObject('MeshTopology', src='@loader', name='container')
    model.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
    model.addObject('UniformMass', totalMass=0.100)
    model.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.45,  youngModulus=600000 / scale)
    model.addObject('LinearSolverConstraintCorrection')

    
    # ##########################################
    # # Effector                               #
    # ##########################################
    effector = model.addChild('effector')
    effector.addObject('MechanicalObject', name="effectorPoint",
                        position=effector_positions,
                        showObject=True,
                        showObjectScale=0.003 * scale,
                        drawMode=1,
                        showColor=[0, 255, 0, 255])
    effector.addObject('PositionEffector', template='Vec3',
                        indices=[0, 1, 2, 3],
                        effectorGoal="@../../goal/goalMO.position")
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)


    ##########################################
    # Visualization                          #
    ##########################################
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=cur_path+"bar.stl", translation=modelTranslation, rotation=modelRotationDegrees, scale=0.001 * scale)
    modelVisu.addObject('OglModel', color=[1., 0., 0., 1.0])
    modelVisu.addObject('BarycentricMapping')


    # ##########################################
    # # Contact                                #
    # ##########################################
    modelContact = model.addChild('contact')
    modelContact.addObject('MeshSTLLoader', name='loader', filename=cur_path+'bar.stl', translation=modelTranslation, rotation=modelRotationDegrees, scale=0.001 * scale)
    modelContact.addObject('MeshTopology', src='@loader', name='topo')
    modelContact.addObject('MechanicalObject')
    modelContact.addObject('TriangleCollisionModel', group=1)
    modelContact.addObject('LineCollisionModel', group=1)
    modelContact.addObject('PointCollisionModel', group=1)
    modelContact.addObject('BarycentricMapping')

    # #########################################
    # # HAND                                   #
    # #########################################

     # hand robot node
    handNode = rootNode.addChild('hand')
    handNode.addObject('EulerImplicitSolver')
    # handNode.addObject('CGLinearSolver', name='Solver', iterations=200)
    handNode.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    handNode.addObject('GenericConstraintCorrection')
    urdfLoader = handNode.addObject('URDFModelLoader', name='URDFModelLoader', filename=urdf_full_filename, 
      modelDirectory=urdf_model_path, useFreeFlyerRootJoint=False, printLog=True, qInit=q_init,
      addCollision=False, addJointsActuators=True)


    ##########################################
    # FINGERS CONTACT                        #
    ##########################################

    # display the 3 actuated fingers frames
    robotNode = handNode.getChild('Robot')
    framesNode = robotNode.getChild('Frames')


    # 0.045 0.05 0.15
    fftipFrameNode = framesNode.getChild(finger_index_tip_frame_name)
    fftipFrameNode.getObject('dof').showObject = True
    fftipFrameNode.getObject('dof').showObjectScale = 0.01

    # 0.045 -0.04 0.15
    lftipFrameNode = framesNode.getChild(finger_pinky_tip_frame_name)
    lftipFrameNode.getObject('dof').showObject = True
    lftipFrameNode.getObject('dof').showObjectScale = 0.01

      # 0.08 0.01 0.12
    thtipFrameNode = framesNode.getChild(finger_thumb_tip_frame_name)
    thtipFrameNode.getObject('dof').showObject = True
    thtipFrameNode.getObject('dof').showObjectScale = 0.01
    

    fingersContact1 = fftipFrameNode.addChild('contact1')
    fingersContact1.addObject('MechanicalObject')
    fingersContact1.addObject('SphereCollisionModel', radius=0.008 * scale, group=2)
    fingersContact1.addObject('RigidMapping', template='Rigid,Vec3d', input="@..", index=0)

    fingersContact2 = lftipFrameNode.addChild('contact2')
    fingersContact2.addObject('MechanicalObject', scale=0.1 * scale)
    fingersContact2.addObject('SphereCollisionModel', radius=0.008 * scale, group=2)
    fingersContact2.addObject('RigidMapping', template='Rigid,Vec3d', input="@..", index=0)
 
    fingersContact3 = thtipFrameNode.addChild('contact3')
    fingersContact3.addObject('MechanicalObject', scale=0.1 * scale)
    fingersContact3.addObject('SphereCollisionModel', radius=0.008 * scale, group=2)
    fingersContact3.addObject('RigidMapping', template='Rigid,Vec3d', input="@..", index=0)


    return rootNode