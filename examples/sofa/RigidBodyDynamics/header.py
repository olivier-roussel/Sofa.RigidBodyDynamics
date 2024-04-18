
def addHeader(rootNode):

    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop') # Needed to use components [FreeMotionAnimationLoop]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [GenericConstraintCorrection]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]  
    
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideForceFields hideWireframe')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName=['Sofa.RigidBodyDynamics_d', 'SofaPython3'])

    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5, printLog=False)
    rootNode.addObject('BackgroundSetting', color=[1., 1., 1., 1.])
    rootNode.findData('dt').value=0.01
    rootNode.gravity = [0,-9810,0]


def createScene(rootNode):

    addHeader(rootNode)
