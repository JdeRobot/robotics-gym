
from setuptools import setup, find_packages
import sys, os.path

# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'robotics-gym'))

setup(name='robotics-gym',
      version='0.0.1',
      install_requires=['gym>=0.2.3', 'pyros_setup'],
      description='The OpenAI Gym for robotics: A toolkit for developing and comparing your reinforcement learning agents using Gazebo, Player/Stage and ROS.',
      url='https://github.com/JdeRobot/robotics-gym',
      author='JdeRobot',
      packages = find_packages(),
      package_data={'robotics-gym/robotics_gym': ['envs/gazebo/assets/launch/*.launch', 'envs/gazebo/assets/worlds/*']},
)