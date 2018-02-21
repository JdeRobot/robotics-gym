import numpy as np
from gym import spaces

import turtlebot_env

class Turtlebot3LaserEnv(turtlebot_env.TurtlebotEnv):

    def __init__(self):
        turtlebot_env.TurtlebotEnv.__init__(self)

        laser = self.get_laser_data()
        self.observation_dims = [len(laser.ranges)]
        self.observation_space = spaces.Discrete(self.observation_dims)
        self.laser = np.zeros(self.observation_dims, np.float32)
        self.action_space = spaces.Discrete(3)
        self.collision = False
        self.obstacle_thresold = 0.15
        self.time_stamp = laser.scan_time

    def _step(self, action):
        self.action2vel(action)

        self.update()

        if self.collision:
            reward = -99
        else:
            if action == 1:
                reward = 0.9
            else:
                reward = -0.003
        info = {}
        return self.laser, reward, self.collision, info

    def _reset(self):

        self.send_velocity_command(0.0, 0.0)
        self.reset_simulation()
        self.update()

        return self.laser, 0.0

    def action2vel(self, action):
        action -= 1

        if action == 0:
            linearx = 0.9
        else:
            linearx = 0.3

        anglz = action * 0.9
        self.send_velocity_command(linearx, anglz)

    def update(self):
        laser = self.get_laser_data()
        self.laser = laser.ranges
        self.time_stamp = laser.scan_time
        self.collision = False
        if np.min(self.laser) < self.obstacle_thresold:
            self.collision = True
