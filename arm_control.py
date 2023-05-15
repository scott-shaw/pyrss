import pybullet as pybl
import numpy as np
import time

def plan_motion_simple(robot, pos, orn):
    ori = pybl.getQuaternionFromEuler(orn)
    target = pybl.calculateInverseKinematics(robot, 4, pos, ori)
    pybl.setJointMotorControlArray(robot, range(5), pybl.POSITION_CONTROL, targetPositions=target)

def plan_motion_joints(robot, joints):
    pybl.setJointMotorControlArray(robot, range(5), pybl.POSITION_CONTROL, targetPositions=joints)

def plan_motion(robot, pos, orn):
    start = [j[0] for j in pybl.getJointStates(robot, range(5))]
    ori = pybl.getQuaternionFromEuler(orn)
    goal = pybl.calculateInverseKinematics(robot, 4, pos, ori)
    return start, goal

def execute_motion(robot, ompl_interface, start, goal):
    robot.set_state(start)
    _, path = ompl_interface.plan(goal)
    #if res:
    ompl_interface.execute(path, dynamics=True)
    return path

def execute_motion_simple(duration, step):
    for _ in range(duration):
        pybl.stepSimulation()
        time.sleep(1./step)

def grab(robot, boxes, box_id):
    #_, cube_orn = pybl.getBasePositionAndOrientation(int(boxes[box_id]))
    cube_orn = pybl.getQuaternionFromEuler([0, np.pi, np.pi])
    id = pybl.createConstraint(robot,5,int(boxes[box_id]),-1,pybl.JOINT_FIXED,jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0.1], childFramePosition=[0, 0, 0], childFrameOrientation=cube_orn)
    pybl.changeConstraint(id, maxForce=999999)
    return id

def release(con_id):
    pybl.removeConstraint(con_id)

