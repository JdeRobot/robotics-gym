import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import turtlebot2_env
import turtlebot2_utils

class Turtlebot2LaserEnv(turtlebot2_env.Turtlebot2Env):

    def __init__(self):
        print('Turtlebot2 laser')
        turtlebot2_env.Turtlebot2Env.__init__(self)


    def _step(self, action):
        
        self.unpause_physics()

        if action == 0: #FORWARD
            self.send_velocity_command(0.3, 0.0)
        elif action == 1: #LEFT
            self.send_velocity_command(0.05, 0.3)
        elif action == 2: #RIGHT
            self.send_velocity_command(0.05, -0.3)

        data = self.get_laser_data()

        self.pause_physics()
        state, done = turtlebot2_utils.discretize_observation(data,5)

        if not done:
            if action == 0:
                reward = 5
            else:
                reward = 1
        else:
            reward = -200

        return state, reward, done, {}

    def _reset(self):

        self.reset_simulation()
        self.unpause_physics()

        data = self.get_laser_data()

        self.pause_physics()
        state = turtlebot2_utils.discretize_observation(data,5)

        return state
