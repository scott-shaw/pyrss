import numpy as np
from env_setup import spawn_box_entities, spawn_robot_arm
import arm_control
import pb_ompl

def rrt_four_box(Yuna):

    NUM_BOXES = 4
    BOX_SPACING = 0.4
    boxes, box_pos, box_orn = spawn_box_entities(NUM_BOXES, row_val=0.8, spacing=BOX_SPACING)

    robot = pb_ompl.PbOMPLRobot(Yuna)
    ompl_interface = pb_ompl.PbOMPL(robot, boxes)
    ompl_interface.set_planner("RRT")

    for i in range(4):
        start, goal = arm_control.plan_motion(Yuna, [box_pos[i,0],box_pos[i,1],0.4], [np.pi,0,0])
        path = arm_control.execute_motion(robot, ompl_interface, start, goal)

        arm_control.plan_motion_simple(Yuna, [box_pos[i,0], box_pos[i,1], 0.2], [np.pi,0,0])
        arm_control.execute_motion_simple(25,16)
        cid = arm_control.grab(Yuna, boxes, i)

        arm_control.plan_motion_joints(Yuna, [0,0,0,0,0])
        arm_control.execute_motion_simple(75,16)

        start, goal = arm_control.plan_motion(Yuna, [-box_pos[i,0],box_pos[i,1],0.4], [np.pi,0,0])
        path = arm_control.execute_motion(robot, ompl_interface, start, goal)
        
        arm_control.plan_motion_simple(Yuna, [-box_pos[i,0], box_pos[i,1], 0.2], [np.pi,0,0])
        arm_control.execute_motion_simple(25,16)
        arm_control.release(cid)
        
        arm_control.plan_motion_joints(Yuna, [0,0,0,0,0])
        arm_control.execute_motion_simple(75,16)


    while(True):
        continue
