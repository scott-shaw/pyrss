import pybullet as pybl
import numpy as np

def spawn_robot_arm():
    YunaStartPos = [0,0,0]
    YunaStartOrientation = pybl.getQuaternionFromEuler([0,0,0])
    Yuna = pybl.loadURDF("urdf/yuna.urdf",YunaStartPos, YunaStartOrientation, useFixedBase=1)
    pybl.configureDebugVisualizer(pybl.COV_ENABLE_RENDERING, 1)
    return Yuna

def gen_around_zero(n, gap):
    arr = []
    if n==0: return arr
    if n%2==0:
        pos_pivot = gap/2
        neg_pivot = -gap/2
    else:
        arr.append(0)
        n -= 1
        pos_pivot = gap
        neg_pivot = -gap
    while n > 0: 
        arr.append(pos_pivot)
        arr.append(neg_pivot)
        pos_pivot += gap
        neg_pivot -= gap
        n -= 2
    return np.sort(arr)

def spawn_box_entities(num_boxes, spacing=0.5, row_val=1, h_val=0):
    boxes = np.empty(num_boxes)
    box_pos = np.empty((num_boxes,3))
    box_orn = np.empty((num_boxes,4))
    spacing_arg = gen_around_zero(num_boxes, spacing)
    print('adding {} boxes at x={}, y={}'.format(num_boxes,row_val,spacing_arg))
    for i in range(num_boxes):
        box_pos[i,:] = [row_val,spacing_arg[i],h_val]
        box_orn[i,:] = pybl.getQuaternionFromEuler([0,0,0])
        boxes[i] = pybl.loadURDF("urdf/box.urdf",box_pos[i], box_orn[i], useFixedBase=0)
    return boxes, box_pos, box_orn

if __name__ == "__main__":
    print(gen_around_zero(3, 0.5))
    print(gen_around_zero(4, 0.5))

