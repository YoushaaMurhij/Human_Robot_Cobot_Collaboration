"""
A Franka Panda reaches for 10 randomly places targets while Youbot reaches for 10 randomly poses
This script contains examples of:
    - Linear (IK) paths.
    - Scene manipulation (creating an object and moving it).
    - Linear mobile paths with an omnidirectional robot to reach a target.
    - A human to walk in between so they can interact with him.
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.mobiles.youbot import YouBot
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape
from pyrep.errors import ConfigurationPathError
import numpy as np
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper


LOOPS = 10
SCENE_FILE = join(dirname(abspath(__file__)), 'colab.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()
arm = Panda()
mobile = YouBot()
#gripper = PandaGripper()

# We could have made this target in the scene, but lets create one dynamically
target = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.05, 0.05, 0.05],
                      color=[1.0, 0.1, 0.1],
                      static=True, respondable=False)
target1 = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.05, 0.05, 0.05],
                      color=[1.0, 0.4, 0.2],
                      static=True, respondable=False)

position_min, position_max = [0.8, 1.8, 0.0], [1.0, 2.2, 0.4]
position_min1, position_max1 = [-0.5, 1.4, 0.1], [1.0, 0.5, 0.1]


starting_joint_positions = arm.get_joint_positions()
starting_pose = mobile.get_2d_pose()

for i in range(LOOPS):

    # Reset the arm at the start of each 'episode'
    #agent.set_joint_positions(starting_joint_positions)
    #agent1.set_2d_pose(starting_pose)

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))
    pos1 = list(np.random.uniform(position_min1, position_max1))

    target.set_position(pos)
    target1.set_position(pos1)

    # Get a path to the target (rotate so z points down)
    try:
        path = arm.get_path(position=pos, euler=[0, math.radians(180), 0])
        path1 = mobile.get_linear_path(position=pos1, angle=0)
        path1.visualize()
    except ConfigurationPathError as e:
        print('Could not find path')
        continue

    # Step the simulation and advance the agent along the path
    done  = False
    done1 = False
    done2 = False
    while not done1:
        if not done:
            done = path.step()
        if not done1:
            done1 = path1.step()
        ##if not done2:
            ##done2 = gripper.actuate(0.5, velocity=0.04)
        pr.step()

    

    path1.clear_visualization()
    print('Reached target %d!' % i)

pr.stop()
pr.shutdown()
