import time
import os
import math
import numpy as np
from numpy.linalg import norm
import pinocchio as pin
# import example_robot_data as robex
from scipy.optimize import fmin_bfgs
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer

urdf_model_path = os.path.dirname(os.path.abspath(__file__)) + '/../robots_description/schunk_hand/'
urdf_file = 'schunk_svh_hand_right_v2.urdf'
urdf_full_filename = os.path.join(urdf_model_path, urdf_file)

enableVisualisation = True

robot = RobotWrapper.BuildFromURDF(urdf_full_filename, package_dirs = urdf_model_path, root_joint=None)

finger_index_tip_frame_name = 'fftip'
finger_pinky_tip_frame_name = 'lftip'
# finger_ring_tip_frame_name = 'rftip'
finger_thumb_tip_frame_name = 'thtip'

scaling = 1.e-2
offset = np.array([0., 0.03, 0.])

def changeCoords(in_vec3):
  # combines rotation -pi/2 arround X then pi/2 arround Z, plus a scaling factor and finally a position offset
  return [in_vec3[2]*scaling, in_vec3[0]*scaling, in_vec3[1]*scaling] + offset
  

# goal_positions = np.array([[4.7, 12, 6], [-1.3, 17.7, 6], [-8.3, 12, 6], [-1.3, 12, 6]])
# have been transformed into
# goal_positions = np.array([[0.06, 0.077, 0.12], [0.06, 0.017, 0.177], [0.06, -0.053, 0.12], [0.06, 0.017, 0.12]])

# effector_positions = np.array([[4.7, 12.0, 5.0], [-1.3, 15.7, 8.0], [-8.3, 12.0, 5.0], [-1.3, 12.0, 5.0]])
# have been transformed into
# effector_positions = np.array([[0.05, 0.077, 0.12 ], [0.08, 0.017, 0.157], [ 0.05, -0.053, 0.12 ], [0.05, 0.017, 0.12 ]])

# for p in effector_positions:
#   pt = changeCoords(p)
#   print("Transformed eff pos = ", pt)

# print("Transformed pos = ", changeCoords(np.array([-2, 1.4, 1.4])))

modelRotation = pin.rpy.matrixToRpy(pin.utils.rotate('x', -math.pi*0.5) @ pin.utils.rotate('z', math.pi*0.5))
print('modelRotation = ', modelRotation)
  
finger_index_tip_frame_idx = robot.model.getFrameId(finger_index_tip_frame_name)
# b = robot.framePlacement(robot.q0, finger_index_tip_frame_idx) # Placement of the end effector tip of index finger
finger_pinky_tip_frame_idx = robot.model.getFrameId(finger_pinky_tip_frame_name)
# finger_ring_tip_frame_idx = robot.model.getFrameId(finger_ring_tip_frame_name)
finger_thumb_tip_frame_idx = robot.model.getFrameId(finger_thumb_tip_frame_name)

q_init = np.array([1.07, 0.023, 1., 0.65,
                   0, 0, 0, 0,
                   0, 0, 0, 0,
                   0, 0, 0, 0,
                   0, 0, 0, 0])

q = q_init

# to_rad = math.pi / 180.

# [2, 15, 4.5] in initial scene
pos_index_target = np.array([0.045, 0.05, 0.15])
# [-7, 15, 4.5] in initial scene
pos_pinky_target = np.array([0.045, -0.04, 0.15])
# [-5, 15, 4.5] in initial scene
# pos_ring_target = changeCoords([-5, 15, 4.5])
# [-2, 12, 8] in initial scene
pos_thumb_target = np.array([0.08, 0.01, 0.12])
# pos_index_target = np.array(finger_index_tip_pose_initial) + offset
# pos_pinky_target = np.array(finger_pinky_tip_pose_initial) + offset
# # pos_ring_target = np.array(finger_ring_tip_pose_initial) + offset
# pos_thumb_target = np.array(finger_thumb_tip_pose_initial) + offset


print("pos_index_target = ", pos_index_target)
print("pos_pinky_target = ", pos_pinky_target)
print("pos_thumb_target = ", pos_thumb_target)
# # [180, -130, 90] + swap Y/Z axis
# rot_index_target = pin.rpy.rpyToMatrix(0.*to_rad, 0.*to_rad, 0.*to_rad) @ (pin.utils.rotate('x', -pi*0.5) @ pin.utils.rotate('z', pi*0.5))
# # [200, -90, 100]
# rot_pinky_target = pin.rpy.rpyToMatrix(200.*to_rad, -90.*to_rad, 100.*to_rad) @ (pin.utils.rotate('x', -pi*0.5) @ pin.utils.rotate('z', pi*0.5))
# rot_ring_target = pin.rpy.rpyToMatrix(0.*to_rad, 0.*to_rad, 0.*to_rad) @ (pin.utils.rotate('x', -pi*0.5) @ pin.utils.rotate('z', pi*0.5))
# # [20, 20, 130]
# rot_thumb_target = pin.rpy.rpyToMatrix(20.*to_rad, 20.*to_rad, 130.*to_rad) @ (pin.utils.rotate('x', -pi*0.5) @ pin.utils.rotate('z', pi*0.5))


# oM_index_target = pin.SE3(rot_index_target, pos_index_target)
# oM_pinky_target = pin.SE3(rot_pinky_target, pos_pinky_target)
# oM_ring_target = pin.SE3(rot_ring_target, pos_ring_target)
# oM_thumb_target = pin.SE3(rot_thumb_target, pos_thumb_target)

if enableVisualisation:
  viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
  viz.initViewer(loadModel=True)
  viz.viewer.open()
  viz.display(robot.q0)
  time.sleep(2.)

def cost(q):
    '''Compute score from a configuration'''
    m_index = robot.framePlacement(q, finger_index_tip_frame_idx)
    p_index = m_index.translation
    d_index = norm(p_index - pos_index_target)**2
    m_pinky = robot.framePlacement(q, finger_pinky_tip_frame_idx)
    p_pinky = m_pinky.translation
    d_pinky = norm(p_pinky - pos_pinky_target)**2
    m_thumb = robot.framePlacement(q, finger_thumb_tip_frame_idx)
    p_thumb = m_thumb.translation
    d_thumb = norm(p_thumb - pos_thumb_target)**2
    
    return d_index + d_pinky + d_thumb


def callback(q):
    viz.display(q)
    time.sleep(1e-2)

q_touch = fmin_bfgs(cost, q_init, callback=callback if enableVisualisation else None)

print("Found inverse kinematics solution for fingers q_touch = ", q_touch)

# pin.framesForwardKinematics(robot.model, robot.data, q_touch)
finger_index_pos_touch = robot.framePlacement(q_touch, finger_index_tip_frame_idx) # Placement of the end effector tip of index finger
finger_pinky_pos_touch = robot.framePlacement(q_touch, finger_pinky_tip_frame_idx) # Placement of the end effector tip of index finger
finger_thumb_pos_touch = robot.framePlacement(q_touch, finger_thumb_tip_frame_idx) # Placement of the end effector tip of index finger

print("Reached position finger_index_pos_touch = ", finger_index_pos_touch)
print("Reached position finger_pinky_pos_touch = ", finger_pinky_pos_touch)
print("Reached position finger_thumb_pos_touch = ", finger_thumb_pos_touch)

# hold on before killing viewer server
if enableVisualisation:
  time.sleep(30.)