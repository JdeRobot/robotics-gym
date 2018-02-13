
from setuptools import setup
import sys, os.path

# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'robotics-gym'))

setup(name='robotics-gym',
      version='0.0.1',
      install_requires=['gym>=0.2.3'],
      description='The OpenAI Gym for robotics: A toolkit for developing and comparing your reinforcement learning agents using Gazebo, Player/Satage and ROS.',
      url='',
      author='JdeRobot',
      package_data={'robotics-gym': ['envs/gazebo/assets/launch/*.launch', 'envs/gazebo/assets/worlds/*']},
)