### A* Algorithm for Path Planning ###

## Usage ##

- aStarNodeBook.ipynb allows the user to experiment with the aStar and other associated methods.
- A sample maze is included (maze.csv)
- aStarDemp.py is a scipt showing a sample/test case

## Structures and Methods ##

# node #
- Written for A* path planning on a 2D grid
- Fields incluide: a) position (tuple) b) parent (node), c) g-cost, d) h-cost, and e) f-cost

# binaryScale(map:np.array, alpha:float) #
- Scales a binary map on the interval (0,1]
- After scaling, all non-zero squares are forced to 1

# getNeighbours(n:node, goal:node, map:np.array) #
- returns a list of navigable neighbours surrounding a map node
- uses 8-point connectivity
- assumes a 2D cartesian grid

# heuristic(start:node, goal:node) #
- calculates the euclidian distance, in index coordinates, between two nodes

# retracePath(n:node, start:node) #
- starting with the end node, retraces the A*-determined path back to the start node.
- first row in output array is the start position (index coordinates)
- last row in outiut array is the end position (index coordinates)

# aStar(start:node, end:node, map:np.array) #
- uses the above functions to implement the A* algorithm

## Dependencies ## 
- Numpy
- OpenCV
- MatPlotLib
- Math

## Resources ##
- https://users.monash.edu/~cema/courses/FIT3094/lecturePDFs/lecture6a_Astar.pdf