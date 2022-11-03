import sys
import time
import collections
import csv
import heapq

class Initiater(object):
    
    drivePath = None
    driveDistance = None
    stateIndex_Heuristics = None
    indexState_Heuristics = None
    def __init__(self):
        self.driveDistance = collections.defaultdict(dict)
        self.starightDistance = collections.defaultdict(dict)
        self.drivePath = "driving.csv"
        self.straightline_file_path = "straightline.csv"
        self.stateIndex_Heuristics = dict()
        self.indexState_Heuristics = dict()
        
        self.Dataset_driving()
        self.Dataset_straight()
        
    def state_to_index_mapping(self):
        return self.stateIndex_Heuristics
    
    def index_to_state_mapping(self):
        return self.indexState_Heuristics
    
    def drive_Distance_val(self):
        return self.driveDistance
    
    def staright_distance_val(self):
        return self.starightDistance
        
    def Dataset_driving(self):
        Index_Variable = 0
        
        file = open(self.drivePath, 'r')
        r = csv.reader(file)
        for row in r:
            if Index_Variable!=0:
                for k in range(1, len(row)):
                    if row[k] != '0' and row[k] !='-1':
                        start = self.getState(Index_Variable)
                        target = self.getState(k)
                        self.driveDistance[start][target] = int(row[k])
                Index_Variable = Index_Variable+1

            elif Index_Variable == 0:
                self.self_mapping(row) 
                Index_Variable=Index_Variable+1
                continue  
                
    def Dataset_straight(self):
        Index_Variable = 0
        
        file = open(self.straightline_file_path, 'r')
        r = csv.reader(file)
        for row in r:
            if Index_Variable!=0:
                for k in range(1, len(row)):
                    start = self.getState(Index_Variable)
                    target = self.getState(k)
                    self.starightDistance[start][target] = int(row[k])
                Index_Variable = Index_Variable+1

            elif Index_Variable == 0:
                Index_Variable=Index_Variable+1
                continue 
        
    def self_mapping(self, row):
        for n in range(1, len(row)):
            self.stateIndex_Heuristics[row[n]] = n
            self.indexState_Heuristics[n] = row[n]
            
    def getState(self, index):
        return self.indexState_Heuristics[index]

class Greedy(object):
    
    driveDistance = None
    starightDistance = None
    root = None
    cost_of_path = None
    r = None
    
    def __init__(self, r):
        self.r = r
        self.root = dict()
        self.driveDistance = self.r.drive_Distance_val()
        self.starightDistance = self.r.staright_distance_val()
        self.cost_of_path = 0
        
    def Greedy(self, source, destination):
        queue = []
        visited_Node = set()
        
        heapq.heappush(queue, [self.distance_value(source, destination), source])
        visited_Node.add(source)
        
        while queue:
            dist, node = heapq.heappop(queue)
            neighbors = self.neighbors_value(node)
            
            if node != destination:
                    for neighbor, dist in neighbors.items():
                      if neighbor not in visited_Node:
                        heapq.heappush(queue, [self.distance_value(neighbor, destination), neighbor])
                        self.root[neighbor] = node
                        visited_Node.add(neighbor)
 
            else:
                print("Target has been reached using Greedy best first search")
                return self.TargetPath(source, destination)
                
    def distance_value(self, start, target):
        return self.starightDistance[start][target]
    
    def neighbors_value(self, node):
        return self.driveDistance[node]

    def TargetPath(self, start, target):
        path = []
        
        while target != start:
            parent = self.root[target]
            self.cost_of_path += self.driveDistance[parent][target]
            path.append(target)
            target = parent
            
        path.append(start)
        return path[::-1]
    
    def costOfPath(self):
        return self.cost_of_path



class astar(object):
    
    driveDistance = None
    starightDistance = None
    root = None
    cost_of_path = None
    r = None
    
    def __init__(self, r):
        self.r = r
        self.root = dict()
        self.driveDistance = self.r.drive_Distance_val()
        self.starightDistance = self.r.staright_distance_val()
        self.cost_of_path = 0
        
    def astar(self, source, destination):
        already_visited_node = []
        visited_Node = set()
        weight = dict()
        funtion = dict()
        
        already_visited_node.append(source)
        weight[source] = 0
        funtion[source] = self.straightLineVal(source, destination)
        
        while len(already_visited_node) > 0:
            present_node = already_visited_node[0]
            p_index = 0
            for i in range(1, len(already_visited_node)):
                if  funtion[already_visited_node[i]] != funtion[present_node] and funtion[already_visited_node[i]] < funtion[present_node]:
                    present_node = already_visited_node[i]
                    p_index = i
                    
            already_visited_node.pop(p_index)
            visited_Node.add(present_node)
            
            if present_node != destination:
                neighbors = self.neighbors_value(present_node)
            else:
                print("Target has been reached using A*")
                return self.TargetPath(source, destination)
                     
            for neighbor, dist in neighbors.items():
                if neighbor in visited_Node:
                    continue
                
                cost = weight[present_node] + dist + self.straightLineVal(neighbor, destination)
                
                if neighbor not in weight or weight[neighbor] > weight[present_node] + dist:
                    weight[neighbor] = weight[present_node] + dist
                    self.root[neighbor] = present_node
                    funtion[neighbor] = cost
                    
                already_visited_node.append(neighbor)
   
                    
    def straightLineVal(self, start, target):
        return self.starightDistance[start][target]
    
    def driveDistanceVal(self, start, target):
        return self.driveDistance[start][target]
    
    def neighbors_value(self, node):
        return self.driveDistance[node]
    
    def TargetPath(self, start, target):
        path = []
        
        while target != start:
            parent = self.root[target]
            self.cost_of_path += self.driveDistance[parent][target]
            path.append(target)
            target = parent
            
        path.append(start)
        return path[::-1]
    
    def costOfPath(self):
        return self.cost_of_path



def main(source, destination):
    r = Initiater()

    states = r.state_to_index_mapping()
    if source not in states or destination not in states:
        print("[NOT FOUND]")
        return

    bfs = Greedy(r)
    startTime = time.time()
    path1 = bfs.Greedy(source, destination)
    timeEnd1 = time.time()
    bfsTime = timeEnd1 - startTime

    aStar = astar(r)
    startTime2 = time.time()
    path2 = aStar.astar(source, destination)
    timeEnd2 = time.time()
    aStarTime = timeEnd2 - startTime2

    print("\n\tKrishnakumar, Pranay Krishna,A20496673 solution:")
    print("\n**************************************************************")
    print("Initial State: ", sys.argv[1])
    print("\nGoal State: ", sys.argv[2])

    print("\nGreedy Best First Search:")
    print("Solution path: ", path1)
    print("Number of States on a Path: ", len(path1))
    print("Path Cost: ", bfs.costOfPath())
    print("Execution time: ", bfsTime)

    print("\nA* Search:")
    print("Solution path: ", path2)
    print("Number of States on a Path: ", len(path2))
    print("Path Cost: ", aStar.costOfPath())
    print("Execution time: ", aStarTime)
    print("\n")

if __name__ == "__main__":

    if len(sys.argv) != 3:
        print("\nERROR: Not enough or too many input arguments\n")
        exit()

    source = sys.argv[1]
    destination = sys.argv[2]

    main(source, destination)