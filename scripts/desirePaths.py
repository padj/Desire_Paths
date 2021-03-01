#%% desirePaths.py
# This script simulates desire paths in a 2D, 
# spatially-discrete, temporally-discrete domain. 

#%% Preamble
import numpy as np
import matplotlib.pyplot as plt
import random
import dP_Astar 

#%% Pseudo-code
# Define environment, spatially-2D
# Spawn random individual
# Assign destination
# Use A* algorithm for pathfinding to the destination
# Once the individual has reached the destination, it leaves 
# the domain.

#%% functions

def defineDomain(m,n, impassableList, pathList):
    # This function defines the 2D spatially-discrete domain. m is the number
    # of points in x, and n is the number of points in y.
    data = {}
    data['x'] = np.linspace(0,m-1,num=m)
    data['y'] = np.linspace(0,n-1,num=n)
    data['xx'], data['yy'] = np.meshgrid(data['x'],data['y'])
    
    data['boundaryAry'] = np.zeros(data['xx'].shape)
    logicBoundaries = (data['xx']==0) | (data['xx']==m-1) | (data['yy']==0) | (data['yy']==n-1)
    data['boundaryAry'][logicBoundaries] = 1 
    
    data['boundaries'] = []
    for i in range(n):
        for j in range(m):
            if data['boundaryAry'][i,j] == 1:
                data['boundaries'].append([i,j])
    
    data['impassableAry'] = np.zeros(data['xx'].shape)
    data['impassables'] = impassableList
    for impass in data['impassables']:
        data['impassableAry'][impass[0]][impass[1]] = 1
    
    data['heatmap'] = np.zeros(data['xx'].shape)
    
    data['paths'] = np.zeros(data['xx'].shape)
    data['pathList'] = pathList
    for path in data['pathList']:
        data['paths'][path[0]][path[1]] = 1
        
    return data



class individual(object):
    def __init__(self, domainDict, targets):
        self.spawn = 0
        self.destination = 0
        while self.spawn == self.destination:
            self.spawn = random.choice(spawns)
            self.destination = random.choice(targets)
            
        self.x = self.spawn[0]
        self.y = self.spawn[1]
        self.location = self.spawn
        self.path = None
        
      

#%% INITIALISATION 

#Define list of impassable nodes
impassables = [[20,42],[21,43],[22,44],[22,42],[21,45],[19,45]]

pathList = [[2,65],[3,65],[4,65],[5,65],[6,65],[7,65],[8,65],[9,64],
            [10,64],[11,64],[12,64],[13,64],[14,64],[15,64],[16,64],[17,63],
            [18,63],[19,63],[20,63],[21,63],[22,63],[23,63],[24,63],[25,62],
            [26,62],[27,62],[28,62],[29,62],[30,62],[31,62],[32,62],[33,61],
            [34,61],[35,61],[36,61],[37,61],[38,61],[39,61],[40,61],[41,60],
            [42,60],[43,60],[44,60],[45,60],[46,60],[47,60],
            [2,40],[3,40],[4,40],[5,40],[6,40],[7,40],[8,40],[9,40],[10,40],
            [11,40],[12,40],[13,40],[14,40],[15,40],[16,40],[17,40],[18,40],
            [19,40],[20,40],[21,40],[22,40],[23,40],[24,40],[25,40],[26,39],
            [27,39],[28,39],[29,39],[30,39],[31,39],[32,39],[33,39],[34,39],
            [35,39],[36,39],[37,39],[38,39],[39,39],[40,39],[41,39],[42,39],
            [43,39],[44,39],[45,39],[46,39],[47,39],
            [2,11],[3,11],[4,11],[5,11],[6,11],[7,12],[8,12],[9,12],[10,12],
            [11,12],[12,13],[13,13],[14,13],[15,13],[16,13],[17,14],[18,14],
            [19,14],[20,14],[21,14],[22,15],[23,15],[24,15],[25,15],[26,15],
            [27,16],[28,16],[29,16],[30,16],[31,16],[32,17],[33,17],[34,17],
            [35,17],[36,17],[37,18],[38,18],[39,18],[40,18],[41,18],[41,19],
            [42,19],[43,19],[44,19],[45,19],[46,19],[47,20],
            [2,45],[3,46],[4,47],[5,48],[6,49],[7,50],[8,51],[9,52],[10,53],
            [11,54],[12,55],[13,56],[14,57],[15,58],[16,59],[17,60],[18,61],
            [19,62],[20,63],[21,64],[22,65],[23,66],[24,67],[25,68],[26,69],
            [27,70],[28,71],[29,72],[30,73],[31,74],[32,75],[33,76],[34,77],
            [35,78],[36,79],[37,80],[38,81],[39,82],[40,83],[41,84],[42,85],
            [43,86],[44,87],[45,88],[46,89],[47,90]]

for i in range(50):
    for j in range(100):
        if i>0 and i<(50-1) and j==(100-2):
            pathList.append([i,j])
        if i>0 and i<(50-1) and j==(1):
            pathList.append([i,j])
        if j>0 and j<(100-1) and i==(1):
            pathList.append([i,j])
        if j>0 and j<(100-1) and i==(50-2):
            pathList.append([i,j])

# Define the domain
domain = defineDomain(100, 50, impassables, pathList)

## DEBUG STUFF
# Define start/end for debug mode. (Used for plot too)
start = [65,1] # Must equal (x,y)
end = [38,47] # Must equal (x,y)
biasCoeff = 0.995 # SET BIAS COEFFICIENT

# Calculate path from start to end using A* algorithm
path = dP_Astar.Astar(start,end,domain)

# Calculate path from start to end using Biased A* algorithm
pathBiased = dP_Astar.AstarBiased(start,end,domain,biasCoeff=biasCoeff)

## POPULATION STUFF
# Define list of possible target/spawn locations.
targets = [[1,1], [28,28], [15,15]]
spawns = [[1,1], [28,28], [15,15]]

#%% Create population of individuals, 

# pop = []
# nPop = 100
# passed = []

# for i in range(nPop):
#     pop.append(individual(domain,targets))
    
# for idv in pop:
#     idv.path = dP_Astar.AstarBiased(idv.spawn, idv.destination, domain, biasCoeff=0.5)
#     if idv.path != None:
#         print("Adding to heatmap")
#         for node in idv.path:
#             domain['heatmap'][node[1]][node[0]] += 1

#%% PLOTS

### PLOT 1
plt.figure(1)
domain['colourMap'] =  domain['boundaryAry'] + (2*domain['impassableAry']+ (3*domain['paths']))
plt.scatter(domain['xx'], domain['yy'], c=domain['colourMap'],cmap='cool')
for node in path:
    plt.scatter(node[0], node[1], marker ='x', s=20, c='k')
plt.scatter(start[0],start[1], s=100, c='r')
plt.scatter(end[0],end[1], s=100, c='r')
plt.title('Domain visualisation')
plt.xlabel('x')
plt.ylabel('y')
plt.axis([0, max(domain['x']), 0, max(domain['y'])])

plt.figure(2)
domain['colourMap'] =  domain['boundaryAry'] + (2*domain['impassableAry']+ (3*domain['paths']))
plt.scatter(domain['xx'], domain['yy'], c=domain['colourMap'],cmap='cool')
for node in pathBiased:
    plt.scatter(node[0], node[1], marker ='o', s=20, c='b')
plt.scatter(start[0],start[1], s=100, c='r')
plt.scatter(end[0],end[1], s=100, c='r')
plt.title('Domain visualisation - %s bias coefficient'%biasCoeff)
plt.xlabel('x')
plt.ylabel('y')
plt.axis([0, max(domain['x']), 0, max(domain['y'])])

# ### PLOT 2
# plt.figure(2)
# plt.scatter(domain['xx'], domain['yy'], c=domain['heatmap'],cmap='binary')
# for node in domain['boundaries']:
#     plt.scatter(node[1],node[0],s=80,c='purple')
# for node in domain['impassables']:
#     plt.scatter(node[1],node[0],s=80,c='magenta')
# plt.title('Domain Heatmap')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.axis([0, max(domain['x']), 0, max(domain['y'])])

# ### PLOT 3
# plt.figure(3)
# domain['colourMap'] =  domain['boundaryAry'] + (2*domain['impassableAry'])
# plt.scatter(domain['xx'], domain['yy'], c=domain['colourMap'],cmap='cool')
# for node in pop[-1].path:
#     plt.scatter(node[0], node[1], marker ='x', s=20, c='k')
# plt.scatter(start[0],start[1], s=100, c='r')
# plt.scatter(end[0],end[1], s=100, c='r')
# plt.title('Domain visualisation')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.axis([0, max(domain['x']), 0, max(domain['y'])])