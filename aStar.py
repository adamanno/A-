import numpy as np
import math
import matplotlib.pyplot as plt
import cv2

class node():

    def __init__(self, parent=None, position=None):
        '''Initialize a node with a parent, position, and g/h/f scores'''
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        '''Equality, allowing for comparison between elements'''
        return self.position == other.position

    def __hash__(self):
        '''Void function allowing for an instance to be hashable'''
        return hash(self.position)

    def updateFCost(self):
        '''Void function that adds internal G and H costs'''
        self.f = self.g + self.h

def binaryScale(map, alpha=1):
    '''A function used to scale a binary map by a decimal factor on (0,1]'''
    w = int(alpha*map.shape[1])
    h = int(alpha*map.shape[0])
    map = cv2.resize(map, (w,h))
    map[map != 0] = 1

    return map

def getNeighbours(n:node, goal:node, map:np.array):
    '''Returns all neighbours of a map square (8-point connectivity)'''
    neighbours = []
    
    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
        # define new position
        x2 = n.position[0] + dx
        y2 = n.position[1] + dy

        # map limits
        xlim = int(map.shape[0])
        ylim = int(map.shape[1])

        if x2 < 0 or x2 >= xlim or y2 < 0 or y2 >= ylim or map[x2][y2] != 0:
            continue # if the position is invalid, discard it
        
        # if node is valid, create the new node and assign scores
        newNeighbour = node(n, (x2,y2))
        newNeighbour.g = heuristic(n, newNeighbour)
        newNeighbour.h = heuristic(n, goal)
        newNeighbour.updateFCost()
        neighbours.append(newNeighbour)
    
    return neighbours

def heuristic(start:node, goal:node):
    '''euclidean distance function'''
    dx = abs(start.position[0] - goal.position[0])
    dy = abs(start.position[1] - goal.position[1])
    return math.sqrt(dx**2 + dy**2)

def retracePath(n:node, start:node):
    '''Recreates the path from the end node. Returns a numpy array'''
    path = []
    curr = n
    
    # until the start node is reched, look back through the nodes
    while curr.parent is not None:
        path.append(curr.position)
        curr = curr.parent
    
    # add start node, reverse list
    path.append(start.position)
    path.reverse()

    # convert to array
    pathArr = np.zeros([len(path),2])
    for i, pos in enumerate(path, 0):
        pathArr[i,:]=[int(pos[0]), int(pos[1])]

    return pathArr


def aStar(start:node, end:node, map:np.array):
    '''A* algorithm, requires a start, an end node, and a binary map as a numpy array'''

    # initilaize with start node
    start.g = 0
    start.h = heuristic(start, end)
    start.updateFCost()

    # create sets, appending start to the open set
    openSet = []
    closedSet = []
    openSet.append(start)

    while len(openSet) > 0:
        # sort based on g (g = h + f), makes implementation easier
        openSet = sorted(openSet, key=lambda n: n.f) 

        # move first ndoe from open set into the closed set
        curr = openSet.pop(0)
        closedSet.append(curr)

        # return path array
        if curr == end:
            p = retracePath(curr, start)
            return p
        
        n:node
        for n in getNeighbours(curr, end, map):
            # only evaluate the neighbour if it is not in the closed set
            if n not in closedSet:
                newG = curr.g + heuristic(curr, n) # evaluate the neighbour's hypothetical g-cost
                
                # if the neighbour is not in the open set, update it's h and f costs 
                # and add it to the open set
                if n not in openSet:
                    n.h = heuristic(n, end)
                    
                    # if the hypotherical g-cost is less than the current g-cost, update parent and 
                    # g-cost
                    if newG < n.g:
                        n.parent = curr
                        n.g = newG
                    
                    n.updateFCost()
                    openSet.append(n)



if __name__ == '__main__':

    map = np.array([[0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1], 
                    [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                    [1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]])

    start = node(None, (0,0))
    end = node(None, (3,10))

    p = aStar(start, end, map)

    x = []
    y = []

    n:node
    for n in p:
        x.append(n[1])
        y.append(n[0])

    fig, ax = plt.subplots()
    ax.imshow(map, 'binary')
    ax.plot(x,y,'-r.')
    ax.plot(start.position[1], start.position[0], 'bo')
    ax.plot(end.position[1], end.position[0], 'bo')

    fig.show()

