import rospy
import numpy as np

def discretize_observation(data, new_ranges, min_range = 0.2):
    discretized_ranges = []
    done = False
    mod = len(data.ranges) / new_ranges
    for i, item in enumerate(data.ranges):
        if (i % mod == 0):
            if data.ranges[i] == float('Inf') or np.isinf(data.ranges[i]):
                discretized_ranges.append(6)
            elif np.isnan(data.ranges[i]):
                discretized_ranges.append(0)
            else:
                discretized_ranges.append(int(data.ranges[i]))
        if (min_range > data.ranges[i] > 0):
            done = True
    return discretized_ranges, done
