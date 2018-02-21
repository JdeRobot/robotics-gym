import numpy as np
from gym import spaces

import turtlebot_env

class Turtlebot2Laser2DEnv(turtlebot_env.TurtlebotEnv):

    def __init__(self):
        turtlebot_env.TurtlebotEnv.__init__(self)

        laser = self.get_laser_data()
        self.observation_dims = [len(laser.ranges), len(laser.ranges)]
        self.observation_space = spaces.Box(low=0, high=255, shape=( self.observation_dims[0], self.observation_dims[1], 1))
        self.screen = np.zeros((self.observation_dims[0], self.observation_dims[1]), np.uint8)
        self.action_space = spaces.Discrete(3)
        self.collision = False
        self.viewer = None
        self.obstacle_thresold = 0.5
        self.time_stamp = laser.scan_time
        self.frames_skip = 0

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
        return self.screen, reward, self.collision, info

    def _reset(self):

        self.send_velocity_command(0.0, 0.0)
        self.reset_simulation()
        self.update()

        return self.screen, 0.0

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

        while self.time_stamp == laser.scan_time:
            pass

        self.collision = False
        if np.min(laser) < self.obstacle_thresold:
            self.collision = True

        for i in range(len(laser.values)):
            value = int((laser.values[i] / 10.0) * self.observation_dims[1])
            self.screen[i] = np.concatenate((np.ones(value) * 255, np.zeros(180 - value)))

        self.time_stamp = laser.scan_time