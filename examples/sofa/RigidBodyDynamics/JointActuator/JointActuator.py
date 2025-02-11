import os

dir_path = os.path.dirname(os.path.abspath(__file__)) + '/'

urdf_file = 'simple_arm.urdf'

urdf_full_filename = os.path.join(dir_path, urdf_file)

# Example of joint actuator usage.
# In this simulation, there is one articulation, on which a servo arm is attached.
# We use the component JointActuator, with PositionEffector to define the desired
# position of the end effector (tip of the arm),
# and the solver QPInverseProblemSolver to solve the inverse problem.


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='Sofa.RigidBodyDynamics')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [UncoupledConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')  # Needed to use components [CGLinearSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting')  # Needed to use components [BackgroundSetting]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')  # Needed to use components [FixedProjectiveConstraint]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Generate')  # Needed to use components [GenerateRigidMass]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshSTLLoader]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')  # Needed to use components [OglModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')

    rootNode.dt = 0.1
    rootNode.gravity = [0., 0., 0.]
    # rootNode.gravity = [0., -9.81, 0.]
    # rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels')
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('FreeMotionAnimationLoop')
    # rootNode.addObject('QPInverseProblemSolver', epsilon=0)
    rootNode.addObject('QPInverseProblemSolver', printLog=True, epsilon=1.e-15, allowSliding=False, actuatorsOnly=True,
                       maxIterations=5000, tolerance=1e-15,
                       qpSolver="proxQP")
                      #  qpSolver="qpOASES")

    # Target position of the end effector
    goal = rootNode.addChild('Goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalEE', position=[0.05, 0.05, 0], # 5cm for initial goal EE position + 4.5cm for offsetting the robot
                   showObject=True, showObjectScale=0.005, drawMode=2)
    goal.addObject('UncoupledConstraintCorrection')
    goal.addObject('SphereCollisionModel', radius=0.006) # for mouse moving

    # Simulation node
    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver')
    simulation.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)

    urdfLoader = simulation.addObject('URDFModelLoader', name='URDFModelLoader', filename=urdf_full_filename, 
      modelDirectory=dir_path, useFreeFlyerRootJoint=False, printLog=True, 
      addCollision=False, addJointsActuators=False)

    robotNode = simulation.getChild('Robot')
    robotNode.addObject('UncoupledConstraintCorrection') # XXX needed ? 

    robotNode.addObject('JointActuator', template='Vec1', index=0,
                     maxAngle=2.5, minAngle=-2.5, maxAngleVariation=0.005)

    framesNode = robotNode.getChild('Frames')
    toolFrameNode = framesNode.getChild('tool0')
    toolFrameNode.getObject('dof').showObject = True
    toolFrameNode.getObject('dof').showObjectScale = 0.01

    effector = toolFrameNode.addChild("Effector")
    effector.addObject("MechanicalObject")
    effector.addObject('RigidMapping', template='Rigid,Vec3d', input="@..", index=0)
    effector.addObject('PositionEffector', indices=0, 
                       effectorGoal=goal.goalEE.findData('position').getLinkPath(),
                       useDirections=[1, 1, 0])

    return rootNode
