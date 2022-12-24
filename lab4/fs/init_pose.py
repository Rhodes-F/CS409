''' Function to initialize agent's poses. 
Input:
    swarmsize --  swarmsize.
    x -- An array to store  agents' x positions, the length of this array is the same as swarmsize
    y -- An array to store agents' y positions, the length of this array is the same as swarmsize
    theta -- An array to store agents' orientations, the length of this is the same as swarmsize

Usage:
    Usr can configure an agent's initial x, y, theta by modifying the value of the corresponding element in array x, y, and theta. 
    For example, initialize agent 0's pose to x = 0, y = 1, theta = 2:
    x[0] = 0
    y[0] = 1
    theta[0] = 2

Constraints to be considered:
    x -- the value should range between -2.5 to 2.5.
    y -- the value should range between -1.5 to 1.5.
    theta -- the value should range between -pi to pi.
    
    The minimal pairwise inter-agent distance should be greater than 0.12
'''
def init(swarmsize, x, y, theta, a_ids):
    import math
    import random
    for i in range(swarmsize):
        x[i] = (i % 10 - 5) * 0.3 + random.uniform(-0.1, 0.1)
        y[i] = (i / 10 - 5) * 0.3 + random.uniform(-0.05, 0.05) + 0.1
        a_ids[i] = i
        theta[i] = random.uniform(-math.pi, math.pi)
    pass
