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
    res, path = ompl_interface.plan(goal)
    #if res:
    ompl_interface.execute(path, dynamics=True)
    return path

def execute_motion_simple(duration, step):
    for _ in range(duration):
        pybl.stepSimulation()
        time.sleep(1./step)

def grab(robot, boxes, box_id):
    return pybl.createConstraint(robot,4,int(boxes[box_id]),-1,pybl.JOINT_FIXED,[0, 0, 0], [0, 0, 0], [0, 0, 0])

def release(con_id):
    pybl.removeConstraint(con_id)

