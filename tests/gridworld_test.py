import os
import gym
import robotics_gym
import time
from robotics_gym.envs import specs

if __name__ == '__main__':

    env = gym.make('SimpleRoomsEnv-v0')
    # env = gym.make('CliffWalkingEnv-v0')
    # env = gym.make('WindyGridworldEnv-v0')
    state = env.reset()

    cont = 0
    while cont != 100:
        cont += 1
        env._render()

    env.close()
