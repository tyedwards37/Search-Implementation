#!/usr/bin/env python
# coding: utf-8

# ## CPSC 390 (Artificial Intelligence): Project 1 - Search
# ## by `Tyler Edwards`
# ## Search using `Uniform Cost Search`
# 
# 
# 

# In[1]:


# Code cell 1: define your common search functions here

from queue import PriorityQueue

# Define the Node class that will be implmented for the search
class Node:
    # Intialize the Node
    def __init__(self, state, parent = None, action = None, pathCost = 0):
        self.state = state
        self.parent = parent 
        self.action = action
        self.pathCost = pathCost
    
    # Expand the current Node
    def expand(self, graph):
        return [self.childNode(action, graph) for action in graph[self.state]]
    
    # Create a return a Child Node based off the information of the current Node
    def childNode(self, action, graph):
        nextState = action
        return Node(nextState, self, action, self.pathCost + graph[self.state][action])
    
    # Returns the total path cost to the final Node which is the Solution Path Cost
    def solution(self):
        return self.path()
    
    # Creates the path from the curreent Node back to the starting Node
    def path(self):
        node, pathBack = self, []
        while node:
            pathBack.append(node.state)
            node = node.parent
        return list(reversed(pathBack))
    
# Define the Uniform Cost Search Algorithm
def uniformCostSearch(start, goal, graph):
    frontier = PriorityQueue()
    startNode = Node(start)
    frontier.put((0, id(startNode), startNode))
    explored = set()
    pathCosts = {start: 0}

    
    while frontier:
        _, _, node = frontier.get()
        
        # Specifies the Node that is being expanded
        print("Expanded: ", node.state)

        # Checks if the current Node is the given goal Node
        if(node.state == goal):
            solutionPath = node.solution()
            totalCost = node.pathCost
            return solutionPath, totalCost 
        
        # Defines the Frontier after every check for the goal Node
        explored.add(node.state)
        for child in node.expand(graph):
            # Checks if the Node has alreayd been explored, then if it hasn't checks to see if there's already a path with a lower cost
            if child.state not in explored and (child.state not in pathCosts or child.pathCost < pathCosts[child.state]):
                frontier.put((child.pathCost, id(child), child))
                pathCosts[child.state] = child.pathCost
        print("Frontier:", [n.state for _, _, n in list(frontier.queue)])

    # Catch for the case where there is no goal Node
    return None, None


# Using `Uniform Cost Search` on the simple tree graph (Figure 1)

# In[2]:


# Code cell 2: Define the graph data of Figure 1 and call the search funciton you implemented in Cell 1

figure1_graph = {
    "S": {"A": 3, "B": 1, "C": 8},
    "A": {"D": 3, "E": 7, "G": 15},
    "B": {"G": 20},
    "C": {"G": 5},
    "D": {},
    "E": {},
    "G": {}
}

# Define the start and goal states
start = "S"
goal = "G"
solution, totalCost = uniformCostSearch(start, goal, figure1_graph)

print("\n")
print("Solution Path: ", solution)
print("Total Path Cost: ", totalCost)


# Using `Uniform Cost Search` on the Romania Roadmap

# In[16]:


# Code cell 3: Define the graph data of Romania Roadmap and call the search funciton you implemented in Cell 1

roadmap = {
    "Arad": {"Zerind": 75, "Sibiu": 140, "Timisoara": 118},
    "Bucharest": {"Urziceni": 85, "Pitesti": 101, "Giurgiu": 90, "Fagaras": 211},
    "Craiova": {"Drobeta": 120, "Rimnicu": 146, "Pitesti": 138},
    "Drobeta": {"Craiova": 120, "Mehadia": 75},
    "Eforie": {"Hirsova": 86},
    "Fagaras": {"Bucharest": 211, "Sibiu": 99},
    "Giurgiu": {"Bucharest": 90},
    "Hirsova": {"Eforie": 86, "Urziceni": 98},
    "Iasi": {"Vaslui": 92, "Neamt": 87},
    "Lugoj": {"Timisoara": 111, "Mehadia": 70},
    "Mehadia": {"Lugoj": 70, "Drobeta": 75},
    "Neamt": {"Iasi": 87},
    "Oradea": {"Zerind": 71, "Sibiu": 151},
    "Pitesti": {"Bucharest": 101, "Rimnicu": 97, "Craiova": 138},
    "Rimnicu": {"Craiova": 146, "Sibiu": 80, "Pitesti": 97},
    "Sibiu": {"Arad": 140, "Fagaras": 99, "Oradea": 151, "Rimnicu": 80},
    "Timisoara": {"Arad":118, "Lugoj": 111},
    "Vaslui": {"Iasi": 92, "Urziceni": 98},
    "Urziceni": {"Vaslui": 142, "Bucharest": 85, "Hirsova": 98},
    "Zerind": {"Arad": 75, "Oradea": 71}
}

# Define the start and goal states
start = "Arad"
goal = "Bucharest"
solution, totalCost = uniformCostSearch(start, goal, roadmap)

print("\n")
print("Solution Path: ", solution)
print("Total Path Cost: ", totalCost)


# Optional: my implementation of A* Search on the Romania roadmap

# In[36]:


# Code cell 4: additional setup for A* search and call the A* search function

# Redefine childNode function so it accomodates for the tuple and that there is an addded priority attribute
class AStarNode(Node):
    def __init__(self, state, parent = None, action = None, path_cost = 0, heuristic = 0):
        super().__init__(state, parent, action, path_cost)
        self.heuristic = heuristic
        self.priority = 0
        
    def childNode(self, action, graph):
        nextState = action
        pathCost, heuristicValue = graph[self.state][action]
        return AStarNode(nextState, self, action, self.pathCost + pathCost, heuristicValue)

# Define the A* Search Algorithm
def aStarSearch(start, goal, graph):
    frontier = PriorityQueue()
    startNode = AStarNode(start)
    startNode.priority = startNode.heuristic
    frontier.put((startNode.priority, id(startNode), startNode))
    explored = set()
    pathCosts = {start: 0}
    
    while frontier:
        _, _, node = frontier.get()
        
        # Specifies the Node that is being expanded
        print("Expanded: ", node.state, "| Heuristic:", node.heuristic)

        # Checks if the current Node is the given goal Node
        if(node.state == goal):
            solutionPath = node.solution()
            totalCost = node.pathCost
            return solutionPath, totalCost 
        
        # Defines the Frontier after every check for the goal Node
        explored.add(node.state)
        for child in node.expand(graph):
            edgeCost, _ = graph[node.state][child.state]
            newCost = node.pathCost + edgeCost
            
            # Checks if the Node has alreayd been explored, then if it hasn't checks to see if there's already a path with a lower cost
            if child.state not in explored and (child.state not in pathCosts or child.pathCost < pathCosts[child.state]):
                pathCosts[child.state] = newCost
                
                # Creates priority based of both heuristic h(n) and least cost g(n)
                priority = newCost + child.heuristic
                frontier.put((child.priority, id(child), child))
        print("Frontier:", [(n.state, n.heuristic) for _, _, n in list(frontier.queue)])

    # Catch for the case where there is no goal Node
    return None, None

roadmap = {
    "Arad": {"Zerind": (75, 374), "Sibiu": (140, 253), "Timisoara": (118, 329)},
    "Bucharest": {"Urziceni": (85, 80), "Pitesti": (101, 100), "Giurgiu": (90, 77), "Fagaras": (211, 176)},
    "Craiova": {"Drobeta": (120, 242), "Rimnicu": (146, 193), "Pitesti": (138, 100)},
    "Drobeta": {"Craiova": (120, 160), "Mehadia": (75, 241)},
    "Eforie": {"Hirsova": (86, 151)},
    "Fagaras": {"Bucharest": (211, 0), "Sibiu": (99, 253)},
    "Giurgiu": {"Bucharest": (90, 0)},
    "Hirsova": {"Eforie": (86, 161), "Urziceni": (98, 80)},
    "Iasi": {"Vaslui": (92, 199), "Neamt": (87, 234)},
    "Lugoj": {"Timisoara": (111, 329), "Mehadia": (70, 241)},
    "Mehadia": {"Lugoj": (70, 244), "Drobeta": (75, 242)},
    "Neamt": {"Iasi": (87, 226)},
    "Oradea": {"Zerind": (71, 374), "Sibiu": (151, 253)},
    "Pitesti": {"Bucharest": (101, 0), "Rimnicu": (97, 193), "Craiova": (138, 160)},
    "Rimnicu": {"Craiova": (146, 160), "Sibiu": (80, 253), "Pitesti": (97, 100)},
    "Sibiu": {"Arad": (140, 366), "Fagaras": (99, 176), "Oradea": (151, 380), "Rimnicu": (80, 193)},
    "Timisoara": {"Arad": (118, 366), "Lugoj": (111, 244)},
    "Vaslui": {"Iasi": (92, 226), "Urziceni": (98, 80)},
    "Urziceni": {"Vaslui": (142, 199), "Bucharest": (85, 0), "Hirsova": (98, 151)},
    "Zerind": {"Arad": (75, 366), "Oradea": (71, 380)}
}

# Define the start and goal states
start = "Arad"
goal = "Bucharest"
solution, totalCost = aStarSearch(start, goal, roadmap)

print("\n")
print("Solution Path: ", solution)
print("Total Path Cost: ", totalCost)
