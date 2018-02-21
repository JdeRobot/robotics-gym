import os
import gym
import robotics_gym
import time
from robotics_gym.envs import specs

if __name__ == '__main__':
    #Turtlebot2
    os.environ['ROBOT_INITIAL_POSE'] = "-x -3.00162 -y -7.99707 -z 0.000693 -R -0.000329 -P -0.020618 -Y 0.001136"
    specs.environment_specs.launchfile = 'turtlebot2_octa.launch'
    env = gym.make('Turtlebot2LaserEnv-v0')
    state = env.reset()
    time.sleep(30)
    env.close()

    #Turtlebot3
    os.environ['TURTLEBOT3_MODEL'] = "burger"
    specs.environment_specs.launchfile = 'turtlebot3_octa.launch'
    env = gym.make('Turtlebot3LaserEnv-v0')
    state = env.reset()
    time.sleep(30)
    env.close()