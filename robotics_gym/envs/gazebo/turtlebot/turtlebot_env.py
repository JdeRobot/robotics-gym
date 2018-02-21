import os
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

from robotics_gym.envs import gazebo_env
from robotics_gym.envs import specs

from gym.utils import seeding
import logging

logger = logging.getLogger(__name__)

class TurtlebotEnv(gazebo_env.GazeboEnv):

    def __init__(self):

        if "TURTLEBOT3_MODEL" not in os.environ:
            self.turtlebot_version = 2
        else:
            self.turtlebot_version = 3

        if 'launchfile' in specs.environment_specs:
            if os.path.exists(specs.environment_specs.launchfile):
                launchfile = specs.environment_specs.launchfile
            elif os.path.exists(os.path.join(os.path.dirname(os.path.realpath(gazebo_env.__file__)),'gazebo/assets/launch/{}'.format(specs.environment_specs.launchfile))):
                launchfile = os.path.join(os.path.dirname(os.path.realpath(gazebo_env.__file__)),'gazebo/assets/launch/{}'.format(specs.environment_specs.launchfile))
            else:
                msg = "TurtlebotEnv: launcfile {} does not exists".format(os.path.exists(specs.environment_specs.launchfile))
                logger.error(msg)
                raise ValueError(msg)
        else:
            if self.turtlebot_version == 2:
                launchfile = "/opt/ros/kinetic/share/turtlebot_gazebo/launch/turtlebot_world.launch"
            else:
                launchfile = "/opt/ros/kinetic/share/turtlebot3_gazebo/launch/turtlebot3_world.launch"                

        if 'port' in specs.environment_specs:
            port = specs.environment_specs.port
            gazebo_env.GazeboEnv.__init__(self, launchfile, port)
        else:
            gazebo_env.GazeboEnv.__init__(self, launchfile)


        if self.turtlebot_version == 3:
            self.vel_pub_service = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        else:
            self.vel_pub_service = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)    

        self.unpause_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_service = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def reset_simulation(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_service()
        except rospy.ServiceException as e:
            logger.warning("TurtlebotEnv: exception raised reset simulation {}".format(e))

    def pause_physics(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_service()
        except rospy.ServiceException as e:
            logger.warning("TurtlebotEnv: exception raised pause physics {}".format(e))

    def unpause_physics(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_service()
        except rospy.ServiceException as e:
            logger.warning("TurtlebotEnv: exception raised unpause physics {}".format(e))

    def get_laser_data(self, timeout=5):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=timeout)
            except Exception as e:
                logger.warning("TurtlebotEnv: exception raised getting laser data {}".format(e))

        return data

    def get_camera_data(self, timeout=5):
        if self.turtlebot_version == 3:
            raise NotImplementedError
            
        image_data = None
        cv_image = None
        while image_data is None:
            try:
                image_data = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=timeout)
                cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
            except Exception as e:
                logger.warning("TurtlebotEnv: exception raised getting camera data {}".format(e))

        return cv_image

    def send_velocity_command(self, linearx, angularz):
        vel_cmd = Twist()
        vel_cmd.linear.x = linearx
        vel_cmd.angular.z = angularz
        self.vel_pub_service.publish(vel_cmd)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _reset(self):
        raise NotImplementedError

    def _step(self, action):
        raise NotImplementedError