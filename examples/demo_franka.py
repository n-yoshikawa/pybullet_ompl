import os.path as osp
import pybullet as p
import math
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import utils

class BoxDemo():
    def __init__(self):
        self.obstacles = []

        p.connect(p.GUI)

        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robotCSC2521HS
        robot_id = p.loadURDF("models/franka_description/robots/panda_arm.urdf", (0,0,0), useFixedBase = 1)
        robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        self.pb_ompl_interface.set_planner("PRM")

        # add obstacles
        self.add_obstacles()

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        pass
        # add box
        #self.add_box([1, 0, 0.7], [0.5, 0.5, 0.05])

        # store obstacles
        #self.pb_ompl_interface.set_obstacles(self.obstacles)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def demo(self):
        start = [0,0,0,-1,0,1.0,0]
        self.robot.set_state(start)

        # Calculate limits
        ll = []
        ul = []
        jr = []
        for bound in self.robot.joint_bounds:
            ll.append(bound[0])
            ul.append(bound[1])
            jr.append(bound[1]-bound[0])
        print("upper limits:", ul)
        print("lower limits:", ll)

        hand_link = 7
        goal_pos = [0.6, 0, 0.2]
        goal_orn = p.getQuaternionFromEuler([math.pi, 0, 0])
        print("goal_orn:", goal_orn)
        goal_state = p.calculateInverseKinematics(self.robot.id, hand_link, goal_pos, goal_orn, lowerLimits=ll, upperLimits=ul, jointRanges=jr)
        goal_state = list(goal_state)
        print("goal_state:", goal_state)
        res, path = self.pb_ompl_interface.plan(goal_state)
        if res:
            x = input("Execute?")
            self.pb_ompl_interface.execute(path)
            x = input("Exit?")
        return res, path

if __name__ == '__main__':
    env = BoxDemo()
    env.demo()
