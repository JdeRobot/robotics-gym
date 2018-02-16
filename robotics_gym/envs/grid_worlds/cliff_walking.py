import sys
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from gym.envs.toy_text import discrete

from matplotlib import pyplot as plt
from matplotlib import pylab
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

""" From edx.org Reinforcement Learning Explained (DAT257x) course """

class CliffWalkingEnv(gym.Env):

    metadata = {'render.modes': ['human','rgb_array', 'ansi']}

    def _limit_coordinates(self, coord):
        coord[0] = min(coord[0], self.shape[0] - 1)
        coord[0] = max(coord[0], 0)
        coord[1] = min(coord[1], self.shape[1] - 1)
        coord[1] = max(coord[1], 0)
        return coord

    def _calculate_transition_prob(self, current, delta):
        new_position = np.array(current) + np.array(delta)
        new_position = self._limit_coordinates(new_position).astype(int)
        new_state = np.ravel_multi_index(tuple(new_position), self.shape)
        reward = -100.0 if self._cliff[tuple(new_position)] else -1.0
        is_done = self._cliff[tuple(new_position)] or (tuple(new_position) == (3,11))
        return [(1.0, new_state, reward, is_done)]

    def __init__(self):
        self.shape = (4, 12)

        nS = np.prod(self.shape)
        nA = 4
        # Cliff Location
        self._cliff = np.zeros(self.shape, dtype=np.bool)
        self._cliff[3, 1:-1] = True

        # Calculate transition probabilities
        P = {}
        for s in range(nS):
            position = np.unravel_index(s, self.shape)
            P[s] = { a : [] for a in range(nA) }
            #UP = 0
            #RIGHT = 1
            #DOWN = 2
            #LEFT = 3
            P[s][0] = self._calculate_transition_prob(position, [-1, 0])
            P[s][1] = self._calculate_transition_prob(position, [0, 1])
            P[s][2] = self._calculate_transition_prob(position, [1, 0])
            P[s][3] = self._calculate_transition_prob(position, [0, -1])

        # We always start in state (3, 0)
        isd = np.zeros(nS)
        isd[np.ravel_multi_index((3,0), self.shape)] = 1.0

        self.P = P
        self.isd = isd                
        self.nS = nS
        self.nA = nA
        self.observation_space = spaces.Discrete(self.nS)
        self.action_space = spaces.Discrete(self.nA)

        self.fig = pylab.figure()
        gs = gridspec.GridSpec(1, 1)
        self.ax = pylab.subplot(gs[:, 0])
        self.ax.xaxis.set_visible(False)
        self.ax.yaxis.set_visible(False)        

        self.ax.set_xticks(np.arange(-.5, 12, 1), minor=True)
        self.ax.set_yticks(np.arange(-.5, 4, 1), minor=True)
        self.ax.grid(which='minor', color='w', linestyle='-', linewidth=1)

        self._imgs_plot = []

        plt.show(False)
        plt.draw()

        self.seed()
        self.reset()

    def _convert_state(self, state):
        converted = np.unravel_index(state, self.shape)
        return np.asarray(list(converted), dtype=np.float32)
    
    def reset(self):
        self.s = np.argmax(self.isd)
        return self._convert_state(self.s)
    
    def step(self, action):
        reward = self.P[self.s][action][0][2]
        done = self.P[self.s][action][0][3]
        info = {'prob':self.P[self.s][action][0][0]}
        self.s = self.P[self.s][action][0][1]
        return (self._convert_state(self.s), reward, done, info)

    def _render(self, mode='rgb_array', close=False):
        if close:
            return

        if mode == 'rgb_array':
            maze = np.zeros((4, 12))
            maze[self._cliff] = -1
            maze[np.unravel_index(self.s, self.shape)] = 2.0
            maze[(3,11)] = 0.5
            img = np.array(maze, copy=True)

            if not hasattr(self, 'imgplot'):
                self.imgplot = self.ax.imshow(img, interpolation='none', cmap='viridis')
            else:
                self.imgplot.set_data(img)

            self._imgs_plot.append([self.imgplot])

            self.fig.canvas.draw()

            return img
        
        else:
            outfile = StringIO() if mode == 'ansi' else sys.stdout

            for s in range(self.nS):
                position = np.unravel_index(s, self.shape)

                if self.s == s:
                    output = " x "
                elif position == (3,11):
                    output = " T "
                elif self._cliff[position]:
                    output = " C "
                else:
                    output = " o "

                if position[1] == 0:
                    output = output.lstrip() 
                if position[1] == self.shape[1] - 1:
                    output = output.rstrip() 
                    output += "\n"

                outfile.write(output)
            outfile.write("\n")

    def create_gif(self, output_path):
        if not len(self._imgs_plot) > 0:
            print('No images')
            return

        fig = plt.figure()
        anim = animation.ArtistAnimation(fig, self._imgs_plot, interval=50, blit=True, repeat_delay=1000)
        anim.save(output_path, writer='imagemagick', fps=4, dpi=80)
        

    def _close(self):
        pass

    def _configure(self):
        pass

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
