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

        # Calculate limits
        self.ll = []
        self.ul = []
        self.jr = []
        self.robot.get_joint_bounds()
        for bound in self.robot.joint_bounds:
            self.ll.append(bound[0])
            self.ul.append(bound[1])
            self.jr.append(bound[1]-bound[0])

        # setup pb_ompl
        #self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        #self.pb_ompl_interface.set_planner("PRM")

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

    def inverse_kinematics(self, pos, orientation, hand_link=7):
        state = p.calculateInverseKinematics(self.robot.id, hand_link, pos, orientation, lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr)
        return list(state)

    def path(self, goal, start=None):
        pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        pb_ompl_interface.set_planner("RRT")

        #print("robot state:", self.robot.state)
        #for i in self.robot.joint_idx:
        #    print(f"getJointState({i}):", p.getJointState(self.robot.id, i))

        if start is None:
            start = self.robot.get_cur_state()
        res, path = pb_ompl_interface.plan_start_goal(start, goal)
        if res:
            pb_ompl_interface.execute(path)
        return res, path

if __name__ == '__main__':
    env = BoxDemo()
    start = [0,0,0,-1,0,1.0,0]
    goal_pos = [0.5, -0.3, 0.3]
    goal_orn = p.getQuaternionFromEuler([math.pi, 1.4, 0])
    goal = env.inverse_kinematics(goal_pos, goal_orn)
    env.path(goal, start)
    goal2_pos = [0.5, 0.3, 0.3]
    goal2 = env.inverse_kinematics(goal2_pos, goal_orn)
    env.path(goal2)
    env.path(start)
