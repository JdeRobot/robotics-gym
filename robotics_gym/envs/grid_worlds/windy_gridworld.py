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

def categorical_sample(prob_n, np_random):
    """
    Sample from categorical distribution
    Each row specifies class probabilities
    """
    prob_n = np.asarray(prob_n)
    csprob_n = np.cumsum(prob_n)
    return (csprob_n > np_random.rand()).argmax()

class WindyGridworldEnv(gym.Env):

    metadata = {'render.modes': ['human','rgb_array', 'ansi']}

    def _limit_coordinates(self, coord):
        coord[0] = min(coord[0], self.shape[0] - 1)
        coord[0] = max(coord[0], 0)
        coord[1] = min(coord[1], self.shape[1] - 1)
        coord[1] = max(coord[1], 0)
        return coord

    def _calculate_transition_prob(self, current, delta, winds):
        new_position = np.array(current) + np.array(delta) + np.array([-1, 0]) * winds[tuple(current)]
        new_position = self._limit_coordinates(new_position).astype(int)
        new_state = np.ravel_multi_index(tuple(new_position), self.shape)
        is_done = tuple(new_position) == (3, 7)
        return [(1.0, new_state, -1.0, is_done)]

    def __init__(self):
        self.shape = (7, 10)

        nS = np.prod(self.shape)
        nA = 4

        # Wind strength
        self.winds = np.zeros(self.shape)
        self.winds[:,[3,4,5,8]] = 1
        self.winds[:,[6,7]] = 2

        # Calculate transition probabilities
        P = {}
        for s in range(nS):
            position = np.unravel_index(s, self.shape)
            P[s] = { a : [] for a in range(nA) }
            #UP = 0
            #RIGHT = 1
            #DOWN = 2
            #LEFT = 3
            P[s][0] = self._calculate_transition_prob(position, [-1, 0], self.winds)
            P[s][1] = self._calculate_transition_prob(position, [0, 1], self.winds)
            P[s][2] = self._calculate_transition_prob(position, [1, 0], self.winds)
            P[s][3] = self._calculate_transition_prob(position, [0, -1], self.winds)

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

    def reset(self):
        self.s = categorical_sample(self.isd, self.np_random)
        self.lastaction=None
        return self.s
    
    def step(self, a):
        transitions = self.P[self.s][a]
        i = categorical_sample([t[0] for t in transitions], self.np_random)
        p, s, r, d= transitions[i]
        self.s = s
        self.lastaction=a
        return (s, r, d, {"prob" : p})

    def _render(self, mode='rgb_array', close=False):
        if close:
            return

        if mode == 'rgb_array':
            maze = np.zeros(self.shape)
            maze[:,[3,4,5,8]] = -0.5
            maze[:,[6,7]] = -1
            maze[np.unravel_index(self.s, self.shape)] = 2.0
            maze[(3,7)] = 0.5
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
                # print(self.s)
                if self.s == s:
                    output = " x "
                elif position == (3,7):
                    output = " T "
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

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
