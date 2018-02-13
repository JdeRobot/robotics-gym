import os
import sys
import gym
from gym import logger
import subprocess

class PlayerStageEnv(gym.Env):

    DEFAULT_PORT = 6665  # default port for master's to bind to

    metadata = {'render.modes': ['human']}

    def __init__(self, configuration_file, port = str(DEFAULT_PORT)):

        if not os.path.exists(configuration_file):
            logger.error("PlayerStageEnv: configuration file not found {}".format(configuration_file))
            sys.exit(-1)
        else:
            logger.info("PlayerStageEnv: configuration file {}".format(configuration_file))

        try:
            subprocess.Popen(["player", "-p", port, configuration_file])
            logger.info("PlayerStageEnv: player launched.")
        except OSError as oe:
            logger.error("PlayerStageEnv: exception raised launching player. {}".format(oe))
            sys.exit(-1)

    def _step(self, action):

        # Implement this method in every subclass
        # Perform a step in gazebo. E.g. move the robot
        raise NotImplementedError

    def _reset(self):

        # Implemented in subclass
        raise NotImplementedError

    def _render(self, mode="human", close=False):
        pass

    def _close(self):

        try:
            ps_output = subprocess.check_output(["ps", "-Af"]).strip("\n")
        except subprocess.CalledProcessError as ce:
            logger.error("PlayerStageEnv: exception raised executing ps command {}".format(ce))
            sys.exit(-1)

        if ps_output.count('player') > 0:
            try:
                subprocess.check_call(["killall","-9", "player"])
                logger.info("PlayerStageEnv: player killed.")
            except subprocess.CalledProcessError as ce:
                logger.warning("PlayerStageEnv: exception raised executing killall command for player {}".format(ce))

    def _configure(self):

        # TODO
        # From OpenAI API: Provides runtime configuration to the enviroment
        # Maybe set the Real Time Factor?
        pass

    def _seed(self):

        # TODO
        # From OpenAI API: Sets the seed for this env's random number generator(s)
        pass