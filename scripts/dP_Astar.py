# A* algorithm

# Takes the following inputs:
    # start location ([x,y])
    # end location ([x,y])
    # domain dictionary (as produced by desirePaths.py)

# Outputs the A* path from start location to end location. 

# Inspo
# https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
# https://gist.github.com/jamiees2/5531924

#%% -------------------------------------- PREAMBLE

import numpy as np

#%% -------------------------------------- CLASS DEFINITIONS
class node(object):
    def __init__(self, location, G, H, parent, passable, boundary):
        self.parent = parent
        self.location = location
        self.G = G
        self.H = H
        self.F = self.G + self.H
        self.passable = passable
        self.boundary = boundary
        
class nodeBiased(object):
    def __init__(self, location, G, H, parent, passable, boundary, paths, 
                                         biasCoeff):
        self.parent = parent
        self.location = location
        self.G = G
        self.H = H
        if paths[self.location[0]][self.location[1]] == 1:
            print('SET PATH FOUND')
            self.P = biasCoeff
        else:
            self.P = 1.0
        self.F = self.G + (self.H*self.P)
        self.passable = passable
        self.boundary = boundary
        

#%% -------------------------------------- FUNCTION DEFINITIONS

def initialisationPhase(domain, start):
    # This function defines the list of lists which describes the grid 
    # environment, and will be populated with nodes.
    NodeMatrix = [[[] for row in domain['x']] for col in domain['y']]
    
    # Create boundary nodes at each boundary, defined by domain.
    for boundary in domain['boundaries']:
        NodeMatrix[boundary[0]][boundary[1]] = node(location=boundary,
                                                    G=1e9, H=1e9,
                                                    parent=None, 
                                                    passable=False,
                                                    boundary=True)
    
    # Create impassable nodes at each impassable, defined by domain.
    for impass in domain['impassables']:
        NodeMatrix[impass[0]][impass[1]] = node(location=impass,
                                                    G=1e9, H=1e9,
                                                    parent=None, 
                                                    passable=False,
                                                    boundary=False)
    
    # Create the start node:
    NodeMatrix[start[0]][start[1]] = node(location=start, G=0, H=0,
                                          parent=None, passable=True,
                                          boundary=False)
    
    return NodeMatrix

def createNeighbourList(location):
    # This function creates a list of loations of neighbours given a starting
    # location. Neighbours are defined based on a first order Moore 
    # neighbourhood
    neighbours = []
    for i in range(-1,2):
            for j in range(-1,2):
                if i == 0 and j == 0:
                    pass
                else:
                    neighbours.append([location[0]+i, location[1]+j])
    
    return neighbours

def populateNeighbours(N,neighbourList, current, currentG, end):
    for ngb in neighbourList:
        #print('current neighbour is %s'%ngb)
        if not N[ngb[0]][ngb[1]]: # if N element is empty
            #print(' - Creating neighbour')
            #print(' - end is %s'%end)
            G = currentG + 1
            #print(' - G is %s'%G)
            H = np.sqrt((end[0]-ngb[0])**2 + (end[1]-ngb[1])**2)
            #print(' - H is %s'%H)
            N[ngb[0]][ngb[1]] = node(location=ngb, G=G, H=H,
                                          parent=current, passable=True,
                                          boundary=False)
        else:
            #print(' - Neighbour exists')
            pass
    return N

def buildPath(N,start,end):
    # This function builds the path from start to end
    path = [] # Define path as empty
    path.append(end) # Add end location to the path
    loc = end # set the location to end.
    while loc != start:
        # while the location is not equal to the start node
        path.append(N[loc[0]][loc[1]].parent) # Add the parent of the location 
        # node to the path list. 
        loc = N[loc[0]][loc[1]].parent # Set the location to the parent.
    
    path.reverse()
    return path
    
def convertPath(oldPath):
    # This function converts the oldPath [row, col] to newPath [x,y]
    
    newPath = []
    for x in oldPath:
        newPath.append([x[1],x[0]])
    
    return newPath

def populateNeighboursBiased(N,neighbourList, current, currentG, end, domain, 
                                          biasCoeff):
    for ngb in neighbourList:
        #print('current neighbour is %s'%ngb)
        if not N[ngb[0]][ngb[1]]: # if N element is empty
            #print(' - Creating neighbour')
            #print(' - end is %s'%end)
            G = currentG + 1
            #print(' - G is %s'%G)
            H = np.sqrt((end[0]-ngb[0])**2 + (end[1]-ngb[1])**2)
            #print(' - H is %s'%H)
            N[ngb[0]][ngb[1]] = nodeBiased(location=ngb, G=G, H=H,
                                          parent=current, passable=True,
                                          boundary=False,
                                          paths=domain['paths'], 
                                          biasCoeff=biasCoeff)
        else:
            #print(' - Neighbour exists')
            pass
    return N

def initialisationPhaseBiased(domain, start, biasCoeff):
    # This function defines the list of lists which describes the grid 
    # environment, and will be populated with nodes.
    NodeMatrix = [[[] for row in domain['x']] for col in domain['y']]
    
    # Create boundary nodes at each boundary, defined by domain.
    for boundary in domain['boundaries']:
        NodeMatrix[boundary[0]][boundary[1]] = nodeBiased(location=boundary,
                                                    G=1e9, H=1e9,
                                                    parent=None, 
                                                    passable=False,
                                                    boundary=True, 
                                                    paths=domain['paths'], 
                                                    biasCoeff=biasCoeff)
    
    # Create impassable nodes at each impassable, defined by domain.
    for impass in domain['impassables']:
        NodeMatrix[impass[0]][impass[1]] = nodeBiased(location=impass,
                                                    G=1e9, H=1e9,
                                                    parent=None, 
                                                    passable=False,
                                                    boundary=False, 
                                                    paths=domain['paths'], 
                                                    biasCoeff=biasCoeff)
    
    # Create the start node:
    NodeMatrix[start[0]][start[1]] = nodeBiased(location=start, G=0, H=0,
                                          parent=None, passable=True,
                                          boundary=False, 
                                          paths=domain['paths'], 
                                          biasCoeff=biasCoeff)
    
    return NodeMatrix    
#%% -------------------------------------- RUN ASTAR PSEUDOCODE



#%% -------------------------------------- RUN ASTAR
# THIS VERSION OF ASTAR IS BASIC NO FRILLS.

def Astar(start, end, domain):
    # ----------------- CONVERT FROM [x,y] to [row,col]
    start = [start[1],start[0]]
    end = [end[1],end[0]]
    
    # ----------------- INITIALISATION
    # Initialise the matrix N, which will be populated with nodes during the 
    # algorithm, and populate N with the starting node
    N = initialisationPhase(domain, start)
    
    # Set the current as starting node
    current = start # current is a list describing a LOCATION.
    currentG = N[current[0]][current[1]].G
    
    # Define the closedList
    closedList = []
    
    #Define the openList
    openList = []
    openList.append(current)

    # Define ended
    ended = False
    
    # ----------------- FIRST LOOP
    
    k= 0 
    kmax = len(domain['xx'])**1.5
    
    # While the openList is not empty and endCondition is False:
    while openList and not ended:
        # Find the node in the openList with the lowest F value, and set 
        # as current.
        current = min(openList, key=lambda x:N[x[0]][x[1]].F) 
    
        # Create a list to define the neighbours.
        neighbourList = createNeighbourList(current)
        
        # Populate N with neighbours of the current node, IF N element is empty.
        N = populateNeighbours(N,neighbourList, current, currentG, end)
        
        # For each neighbour in the list:
        for ngb in neighbourList:
            # if the neighbour is boundary, impassable or on closedList -> ignore.
            if (N[ngb[0]][ngb[1]].boundary == True or 
                    N[ngb[0]][ngb[1]].passable == False or
                    ngb in closedList):
                pass
            
            # if the neighbour already existed (i.e. already in openList):
            elif ngb in openList == True:
                # Check whether the new path to this node is better
                if N[ngb[0]][ngb[1]].G < currentG+1:
                    # If better: reset G, set current as parent, recalc F
                    N[ngb[0]][ngb[1]].G = currentG+1
                    N[ngb[0]][ngb[1]].parent = current
                    N[ngb[0]][ngb[1]].F = N[ngb[0]][ngb[1]].G + N[ngb[0]][ngb[1]].H
            
            # if the neighbour isn't in the openlist:
            else:
                # Set the parent as current and determine G,H,F. This is done in
                # populationNeighbours()
                # Add the neighbour to the openList.
                openList.append(ngb)
        
        # Remove the current node from the openList.
        openList.remove(current)
        
        # Add the current node to the closedList.
        closedList.append(current)
        
        if end in closedList:
            ended = True # Set end criteria
            print('Target found')
            path = buildPath(N, start, end) # create a list of the path from
            # start to end.
            # Convert the path from [row,col] back to [x,y]
            path = convertPath(path)
            
        elif k > kmax:
            ended = True
            print('Target NOT found. k exceeded')
            path = None
        
        k += 1
    
    return path
    
#%% -------------------------------------- RUN ASTAR inc. PATHS
# THIS VERSION OF ASTAR BUILDS ON THE BASIC VERSION, BY INCLUDING BIASING 
# TOWARDS PATHS TAKEN BY OTHER INDIVIDUALS.
# This is done by including an additional component to the F metric, called P.
# P is a measure of the footfall of a node, calculated as:
# P = T/

def AstarBiased(start, end, domain, biasCoeff):
    # ----------------- CONVERT FROM [x,y] to [row,col]
    start = [start[1],start[0]]
    end = [end[1],end[0]]
    
    # ----------------- INITIALISATION
    # Initialise the matrix N, which will be populated with nodes during the 
    # algorithm, and populate N with the starting node
    N = initialisationPhaseBiased(domain, start, biasCoeff = biasCoeff)
    
    # Set the current as starting node
    current = start # current is a list describing a LOCATION.
    currentG = N[current[0]][current[1]].G
    
    # Define the closedList
    closedList = []
    
    #Define the openList
    openList = []
    openList.append(current)

    # Define ended
    ended = False
    
    # ----------------- FIRST LOOP
    
    k= 0 
    kmax = len(domain['xx'])**1.5
    
    # While the openList is not empty and endCondition is False:
    while openList and not ended:
        # Find the node in the openList with the lowest F value, and set 
        # as current.
        current = min(openList, key=lambda x:N[x[0]][x[1]].F) 
    
        # Create a list to define the neighbours.
        neighbourList = createNeighbourList(current)
        
        # Populate N with neighbours of the current node, IF N element is empty.
        N = populateNeighboursBiased(N,neighbourList, current, currentG, end, 
                                     domain, biasCoeff)
        
        # For each neighbour in the list:
        for ngb in neighbourList:
            # if the neighbour is boundary, impassable or on closedList -> ignore.
            if (N[ngb[0]][ngb[1]].boundary == True or 
                    N[ngb[0]][ngb[1]].passable == False or
                    ngb in closedList):
                pass
            
            # if the neighbour already existed (i.e. already in openList):
            elif ngb in openList == True:
                # Check whether the new path to this node is better
                if N[ngb[0]][ngb[1]].G < currentG+1:
                    # If better: reset G, set current as parent, recalc F
                    N[ngb[0]][ngb[1]].G = currentG+1
                    N[ngb[0]][ngb[1]].parent = current
                    N[ngb[0]][ngb[1]].F = N[ngb[0]][ngb[1]].G + N[ngb[0]][ngb[1]].H
            
            # if the neighbour isn't in the openlist:
            else:
                # Set the parent as current and determine G,H,F. This is done in
                # populationNeighbours()
                # Add the neighbour to the openList.
                openList.append(ngb)
        
        # Remove the current node from the openList.
        openList.remove(current)
        
        # Add the current node to the closedList.
        closedList.append(current)
        
        if end in closedList:
            ended = True # Set end criteria
            print('Target found')
            path = buildPath(N, start, end) # create a list of the path from
            # start to end.
            # Convert the path from [row,col] back to [x,y]
            path = convertPath(path)
            
        elif k > kmax:
            ended = True
            print('Target NOT found. k exceeded')
            path = buildPath(N, start, current) # create a list of the path from
            # start to end.
            # Convert the path from [row,col] back to [x,y]
            path = convertPath(path)
        
        k += 1
    
    return path