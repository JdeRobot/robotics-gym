import os
import sys
import gym
import logging
import rospy
import subprocess

logger = logging.getLogger(__name__)

class GazeboEnv(gym.Env):

    DEFAULT_NODE_PORT = 0  # bind to any open port
    DEFAULT_MASTER_PORT = 11311  # default port for master's to bind to
    DEFAULT_MASTER_URI = 'http://localhost:%s/' % DEFAULT_MASTER_PORT

    metadata = {'render.modes': ['human']}

    def __init__(self, launch_file, port = str(DEFAULT_MASTER_PORT)):

        if not os.path.exists(launch_file):
            logger.error("GazeboEnv: launch file not found {}".format(launch_file))
            sys.exit(-1)
        else:
            logger.info("GazeboEnv: launch file {}".format(launch_file))

        try:
            with open(".roscore_stdout.log", "w") as out, open(".roscore_stderr.log", "w") as err:
                subprocess.Popen(["roscore", "-p", port], stdout=out, stderr=err)
            logger.info("GazeboEnv: roscore launched.")
        except OSError as oe:
            logger.error("GazeboEnv: exception raised launching roscore. {}".format(oe))
            sys.exit(-1)

        try:
            rospy.init_node('gym', anonymous=True)
            logger.info("GazeboEnv: gym ROS node created.")
        except rospy.exceptions.ROSException as re:
            logger.error("GazeboEnv: exception raised creating gym ROS node. {}".format(re))
            self._close()
            sys.exit(-1)

        try:
            with open(".roslaunch_stdout.log", "w") as out, open(".roslaunch_stderr.log", "w") as err:
                subprocess.Popen(["roslaunch", "-p", port, launch_file], stdout=out, stderr=err)
            logger.info("GazeboEnv: gzserver launched.")
        except OSError as oe:
            logger.error("GazeboEnv: exception raised launching gzserver. {}".format(oe))
            self._close()
            sys.exit(-1)

    def _step(self, action):
        raise NotImplementedError

    def _reset(self):
        raise NotImplementedError

    def _render(self, mode="human", close=False):

        try:
            ps_output = subprocess.check_output(["ps", "-Af"]).strip("\n")
        except subprocess.CalledProcessError as ce:
            logger.warning("GazeboEnv: exception raised executing ps command {}".format(ce))
            ps_output = ''

        if ps_output.count('gzclient') > 0 and close:
            try:
                subprocess.check_call(["killall","-9", "gzclient"])
                logger.info("GazeboEnv: gzclient killed.")
            except subprocess.CalledProcessError as ce:
                logger.warning("GazeboEnv: exception raised executing killall command for gzclient {}".format(ce))
            return

        if mode == 'human' and ps_output.count('gzclient') < 1:
            try:
                subprocess.Popen(["gzclient"])
                logger.info("GazeboEnv: gzclient launched.")
            except OSError as oe:
                logger.warning("GazeboEnv: exception raised launching gzclient {}".format(oe))

    def _close(self):

        try:
            ps_output = subprocess.check_output(["ps", "-Af"]).strip("\n")
        except subprocess.CalledProcessError as ce:
            logger.error("GazeboEnv: exception raised executing ps command {}".format(ce))
            sys.exit(-1)

        if ps_output.count('gzclient') > 0:
            try:
                subprocess.check_call(["killall","-9", "gzclient"])
                logger.info("GazeboEnv: gzclient killed.")
            except subprocess.CalledProcessError as ce:
                logger.warning("GazeboEnv: exception raised executing killall command for gzclient {}".format(ce))

        if ps_output.count('gzserver') > 0:
            try:
                subprocess.check_call(["killall","-9", "gzserver"])
                logger.info("GazeboEnv: gzserver killed.")
            except subprocess.CalledProcessError as ce:
                logger.warning("GazeboEnv: exception raised executing killall command for gzserver {}".format(ce))

        if ps_output.count('rosmaster') > 0:
            try:
                subprocess.check_call(["killall","-9", "rosmaster"])
                logger.info("GazeboEnv: rosmaster killed.")
            except subprocess.CalledProcessError as ce:
                logger.warning("GazeboEnv: exception raised executing killall command for rosmaster {}".format(ce))

        if ps_output.count('roscore') > 0:
            try:
                subprocess.check_call(["killall","-9", "roscore"])
                logger.info("GazeboEnv: roscore killed.")
            except subprocess.CalledProcessError as ce:
                logger.warning("GazeboEnv: exception raised executing killall command for roscore {}".format(ce))

    def _configure(self):
        pass

    def _seed(self):
        pass