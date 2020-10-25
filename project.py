# -*- coding: utf-8 -*-
"""
Created on Wed Oct 10 17:14:16 2018

@author: shayanSM
"""
import math



"""
Artificial Intelligence
Computer Assignment NO.1
student : Shayan Shafiee Moghadam
studentID : 810994076
major : Industrial engineering
"""



"""
this class represents all the nodes in the text files
this class contains :
    1) node id --> nodeID
    2) latitude --> y
    3) longitude --> x
    4) a dictionary for distance --> distance
    5) a dictionary for speed --> speed
    6) a dictionary for one way or two way roads --> accessible
"""
class Node(object):
    
    
    # initial values : nodeID , latitude , longitude
    # assign 3 empty dictionaries : speed , distance , accessible
    def __init__(self,nodeID,y,x):
        self.nodeID = nodeID
        self.y = y
        self.x = x
        self.distance = dict()
        self.speed = dict()
        self.accessible = dict()
    
    
    # add neighbors or roads to node
    # calculate the distance
    # p.s. distances do not have any standard value and units
    # however, they can be used for comparison
    def roadAdd(self,neighbor,max_speed,accessiblity):
        # neighbor is an object 
        # max speed and accesibility are numbers
        self.speed[neighbor.nodeID] = max_speed
        self.accessible[neighbor.nodeID] = accessiblity
        coords_1 = (self.x, self.y)
        coords_2 = (neighbor.x, neighbor.y)
        self.distance[neighbor.nodeID] = Node.distance(coords_1, coords_2)

    
    @staticmethod
    def distance(origin, destination):
        lat1, lon1 = origin
        lat2, lon2 = destination
        radius = 6371  # km
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlon / 2) * math.sin(dlon / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = radius * c
        return d
    
    
    # printing the object
    def __str__(self):
        string = ''
        for i in self.speed.keys():
            string += "neighbor " + str(i) + " with max speed " + str(self.speed[i]) + " and distance " + str(self.distance[i]) + " is "
            if self.accessible[i] == 0:
                string += "not "
            string += "accessible\n"
        return "node id : %s \nlatitude : %s\nlongitude : %s\n%s" % (self.nodeID,self.y,self.x,string)
    

"""
this class represent all the nodes in the tree search in problem 1
this class contains :
    1) node id --> nodeID
    2) parent id --> parentID
    3) distance between start node to current nore --> cost
    4) depth of the node --> depth 
"""
class TreeNode (object):
    
    
    # this method will be used if we want to define a root
    # parent id for root is -1
    # cost and depth of the root are zero
    def initialState(self,start_node_id):
        self.nodeID = start_node_id
        self.parentID = -1
        self.cost = 0
        self.depth = 0
        # the treeNodes dictionary is empty 
        # in the following the root will be added to the treeNodes
        global treeNodes
        treeNodes[start_node_id] = self
        
    
    # in this method, neighbors of the node will be added to the treeNodes dictionary
    def createNeighbor(self,neighborID):
        node = nodes[self.nodeID]
        neighbor = TreeNode()
        neighbor.nodeID = neighborID
        neighbor.parentID = self.nodeID
        neighbor.cost = self.cost + node.distance[neighborID]
        neighbor.depth = self.depth + 1
        global treeNodes
        treeNodes[neighborID] = neighbor
    
    
    def is_goal(self,target_node_id):
        return self.nodeID == target_node_id
    
    
    def __str__(self):
        return "node id : %s \nparent id : %s\ncost : %s\ndepth : %s" % (self.nodeID,self.parentID,self.cost,self.depth)



"""
this class represent all the nodes in the tree search in problem 2
it is like the first class, however it uses the cost function in a different way
this class is a subclass of the TreeNodes which have changed the cost function 
"""
class TreeNode2 (TreeNode):
    
    
    def createNeighbor(self,neighborID):
        node = nodes[self.nodeID]
        neighbor = TreeNode2()
        neighbor.nodeID = neighborID
        neighbor.parentID = self.nodeID
        neighbor.cost = self.cost + (node.distance[neighborID]/node.speed[neighborID])
        neighbor.depth = self.depth + 1
        global treeNodes
        treeNodes[neighborID] = neighbor



"""
A Queue is a linear structure which follows a particular order in which the
operations are performed. The order is First In First Out (FIFO)
"""
class queue(object):    
    
    
    __list = []
    
    
    def enqueue(self,insertion):
        self.__list += [insertion]
        
        
    def dequeue(self):
        if self.__list != [] :
            deletion = self.__list[0]
            self.__list.pop(0)
            return deletion
        return False
    

    def isEmpty(self):
        if self.__list == [] :
            return True
        return False
    
    
    def isIn(self,insertion):
        return insertion in self.__list



"""
Stack is a linear data structure which follows a particular order in which the 
operations are performed. The order may is LIFO(Last In First Out).
"""
class stack(object):    
    
    
    __list = []
    
    
    def add(self,insertion):
        self.__list += [insertion]
        
        
    def remove(self):
        if self.__list != [] :
            deletion = self.__list[0] # -1
            self.__list.pop(0) #-1
            return deletion
        return False
    

    def isEmpty(self):
        return self.__list == []
    
    
    def isIn(self,insertion):
        return insertion in self.__list



"""
"""
class priorityQueue(object):
    
    
    __dict = dict()
    
    
    def isEmpty(self):
        if len(self.__dict) == 0 :
            return True
        return False
    
    
    def isAddable(self,insertion,cost):
        if insertion not in self.__dict.keys():
            return True
        if self.__dict[insertion] > cost:
            del self.__dict[insertion]
            return True
        return False
       
        
    def insert(self,insertion,cost):
        self.__dict[insertion] = cost
    
    
    def deleteMin(self):
        if len(self.__dict) == 0 :
            return False
        minCost = min(self.__dict.values())
        for i in self.__dict.keys():
            if self.__dict[i] == minCost:
                del self.__dict[i]
                return i
    
    
    
def readtext():
    
    
    nodes = dict()
    fileNodes = open("nodes.txt","r")
    infoNodes = fileNodes.readlines()
    for i in infoNodes:
        string = i.split(" ")
        nodeID = int(string[0])
        y = float(string[1])
        x = float(string[2])
        newNode = Node(nodeID,y,x)
        nodes [nodeID] = newNode
        
        
    fileEdges = open("edges.txt","r")
    infoEdges = fileEdges.readlines()
    for i in infoEdges:
        string = i.split(" ")
        node1ID = int(string[0])
        node2ID = int(string[1])
        accessibility1 = 1
        accessibility2 = 1
        max_speed = float(string[2])
        is_one_way = int(string[3])
        if is_one_way == 1:
            accessibility1 = 0
        nodes[node1ID].roadAdd(nodes[node2ID],max_speed,accessibility2)
        nodes[node2ID].roadAdd(nodes[node1ID],max_speed,accessibility1)
        
        
    return nodes



"""
this function calculates the maximum of max_speed
the return value will be used for the heuristic function
"""
def maxSpeedClaculate():
    fileEdges = open("edges.txt","r")
    infoEdges = fileEdges.readlines()
    a = []
    for i in infoEdges:
        string = i.split(" ")
        max_speed = float(string[2])
        a += [max_speed]
    maxSpeed = max(a)
    return maxSpeed



"""
Global variables:
nodes variable represents the information of each location (node)
treeNodes variable saves the information of states in search tree
"""
# this dictionary contains all the information of nodes
# the key for finding a node (object) is its id
nodes = readtext()
# this dictionary contain all the nodes which are appearing in the search space
treeNodes = dict()

maxSpeed = maxSpeedClaculate()


"""
breadth first search function
"""
def bfs(start_node_id, target_node_id):
    visited = 0
    treeNodes.clear()
    root = TreeNode()
    root.initialState(start_node_id)
    frontier = queue()
    frontier.enqueue(start_node_id)
    explored = set()
    while not frontier.isEmpty():
        stateID = frontier.dequeue()
        visited += 1
        explored.add(stateID)
        if treeNodes[stateID].is_goal(target_node_id):
            break
        #print(nodes[stateID])
        #print(stateID)
        #print(treeNodes[stateID])
        # adding naighbor
        for neighbor in nodes[stateID].distance.keys():
            if (neighbor not in explored) and (not frontier.isIn(neighbor)):
                treeNodes[stateID].createNeighbor(neighbor)
                frontier.enqueue(neighbor)
                
    
    # display solution            
    print("visited:" , visited)
    print("path:", treeNodes[stateID].depth)
    print("distance:",treeNodes[stateID].cost)
    path = []
    while treeNodes[stateID].parentID != -1:
        path += [stateID]
        stateID = treeNodes[stateID].parentID
    path += [stateID]
    print(path[::-1])



"""
uniform cost search function
"""
def ucs(start_node_id, target_node_id):
    visited = 0
    treeNodes.clear()
    root = TreeNode()
    root.initialState(start_node_id)
    frontier = priorityQueue()
    frontier.insert(start_node_id,treeNodes[start_node_id].cost)
    explored = set()
    while not frontier.isEmpty():
        stateID = frontier.deleteMin()
        visited += 1
        explored.add(stateID)
        if treeNodes[stateID].is_goal(target_node_id):
            break
        #print(nodes[stateID])
        #print(stateID)
        #print(treeNodes[stateID])
        # adding naighbor
        for neighbor in nodes[stateID].distance.keys():
            neighborCost = treeNodes[stateID].cost + nodes[stateID].distance[neighbor]
            if (neighbor not in explored) and (frontier.isAddable(neighbor,neighborCost)):
                treeNodes[stateID].createNeighbor(neighbor)
                #print (neighborCost == treeNodes[neighbor].cost)
                frontier.insert(neighbor,treeNodes[neighbor].cost)
        # display solution            
    print("visited:" , visited)
    print("path:", treeNodes[stateID].depth)
    print("distance:",treeNodes[stateID].cost)
    path = []
    while treeNodes[stateID].parentID != -1:
        path += [stateID]
        stateID = treeNodes[stateID].parentID
    path += [stateID]
    print(path[::-1])

"""
Depth limited search function
wont work optimally for bigger numbers (>10) like bfs
"""
visited = 0
def dls(start_node_id, target_node_id,limit):
    treeNodes.clear()
    root = TreeNode()
    root.initialState(start_node_id)
    frontier = stack()
    frontier.add(start_node_id)
    explored = set()
    while not frontier.isEmpty():
        stateID = frontier.remove()
        global visited
        visited += 1
        explored.add(stateID)
        if treeNodes[stateID].is_goal(target_node_id):
            return True
        #print(stateID)
        # adding naighbor
        if treeNodes[stateID].depth < limit:
            for neighbor in nodes[stateID].distance.keys():
                if (neighbor not in explored) and (not frontier.isIn(neighbor)):
                    treeNodes[stateID].createNeighbor(neighbor)
                    frontier.add(neighbor)
    return False



"""
Iterative Deepening function
"""
def id(start_node_id, target_node_id):
    depth = 0
    while not dls(start_node_id, target_node_id,depth):
        depth += 1
    print("visited:" , visited)
    print("path:", treeNodes[target_node_id].depth)
    print("distance:",treeNodes[target_node_id].cost)
    path = []
    stateID = target_node_id
    while treeNodes[stateID].parentID != -1:
        path += [stateID]
        stateID = treeNodes[stateID].parentID
    path += [stateID]
    print(path[::-1])
    
"""
admissible heuristic function
"""
def heuristic(currentState,goalState):
    x1 = currentState.x
    y1 = currentState.y
    x2 = goalState.x
    y2 = goalState.y
    sqrtx = (x1-x2)**2
    sqrty = (y1-y2)**2
    distance = (sqrtx + sqrty)**(0.5)
    return (distance / maxSpeed)



"""
A* search algorithm
"""    
def astar(start_node_id, target_node_id):
    visited = 0
    treeNodes.clear()
    root = TreeNode2()
    root.initialState(start_node_id)
    frontier = priorityQueue()
    frontier.insert(start_node_id,treeNodes[start_node_id].cost)
    explored = set()
    while not frontier.isEmpty():
        stateID = frontier.deleteMin()
        visited += 1
        explored.add(stateID)
        if treeNodes[stateID].is_goal(target_node_id):
            break
        #print(nodes[stateID])
        #print(stateID)
        #print(treeNodes[stateID])
        # adding naighbor
        for neighbor in nodes[stateID].distance.keys():
            time = nodes[stateID].distance[neighbor]/nodes[stateID].speed[neighbor]
            neighborCost = treeNodes[stateID].cost + time + heuristic(nodes[stateID],nodes[neighbor])
            if (neighbor not in explored) and (frontier.isAddable(neighbor,neighborCost)):
                if nodes[stateID].accessible[neighbor] == 1:
                    treeNodes[stateID].createNeighbor(neighbor)
                    #print (neighborCost == treeNodes[neighbor].cost)
                    frontier.insert(neighbor,neighborCost)
        
    # display solution            
    print("visited:" , visited)
    print("path:", treeNodes[stateID].depth)
    print("distance:",treeNodes[stateID].cost)
    path = []
    while treeNodes[stateID].parentID != -1:
        path += [stateID]
        stateID = treeNodes[stateID].parentID
    path += [stateID]
    print(path[::-1])  