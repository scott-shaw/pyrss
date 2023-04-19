# https://github.com/lyfkyle/pybullet_ompl

import pybullet as pybl
import pybullet_data
import numpy as np
import os
import time
from env_setup import spawn_box_entities, spawn_robot_arm
import arm_control
import pb_ompl
from demos import rrt_four_box

physicsClient = pybl.connect(pybl.GUI, options='--background_color_red=0.2 --background_color_green=0.2 --background_color_blue=0.2')
# set visualization
pybl.configureDebugVisualizer(pybl.COV_ENABLE_RENDERING, 0)
pybl.configureDebugVisualizer(pybl.COV_ENABLE_GUI, 0)

# load world
pybl.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF (plane)
planeId = pybl.loadURDF("plane.urdf")
pybl.setGravity(0,0,-9.81)
pybl.setRealTimeSimulation(0)

# load robot model
Yuna = spawn_robot_arm()

'''
box_orn = pybl.getQuaternionFromEuler([0,0,0])
box = pybl.loadURDF("urdf/box.urdf",[1,1,0], box_orn, useFixedBase=0)
arm_control.plan_motion_joints(Yuna, [np.pi/4,np.pi/6,np.pi/6,np.pi/4,np.pi/3])
arm_control.execute_motion_simple(75,16)
while True:
    continue
'''

time.sleep(5)
rrt_four_box(Yuna)

# ===================== RUN =====================
'''
for i in range(8):

    arm_control.plan_motion(Yuna, [box_pos[i,0],box_pos[i,1],0.2], [np.pi,0,0])
    arm_control.execute_motion(75,16)
    cid = arm_control.grab(Yuna, boxes, i)

    arm_control.plan_motion(Yuna, [box_pos[i,0],box_pos[i,1],0.5], [np.pi,0,0])
    arm_control.execute_motion(25,16)

    arm_control.plan_motion(Yuna, [-box_pos[i,0],0.6,0.2], [np.pi,0,0])
    arm_control.execute_motion(75,16)
    arm_control.release(cid)

    arm_control.plan_motion(Yuna, [-box_pos[i,0],0.6,0.5], [np.pi,0,0])
    arm_control.execute_motion(25,16)
'''
#arm_control.plan_motion(Yuna, [0.8,0.6,0.2], [np.pi,0,0])
#arm_control.execute_motion(75,16)
#cid = arm_control.grab(Yuna, boxes, 7)

#arm_control.plan_motion(Yuna, [-0.8,0.6,0.2], [np.pi,0,0])
#arm_control.execute_motion(75,16)
#arm_control.release(cid)

#arm_control.plan_motion(Yuna, [0,0,0], [0,0,0])
#arm_control.execute_motion(75,16)

pybl.disconnect()

