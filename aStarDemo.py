import numpy as np
from aStar import aStar, binaryScale, node
import matplotlib.pyplot as plt

if __name__ == '__main__':
    map = np.genfromtxt('maze.csv', delimiter=',')


    # create a subset of the map
    map = map[0:180,290:450]
    start = node(None, (98,33)) # start node
    end = node(None, (98,126)) # goal

    p = aStar(start, end, map)

    # plot data
    fig, ax = plt.subplots()
    ax.imshow(map, 'binary')
    ax.plot(start.position[1], start.position[0], 'b.')
    ax.plot(end.position[1], end.position[0], 'g.')
    ax.plot(p[:,1], p[:,0], 'r-')