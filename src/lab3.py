#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped
from numpy import ma
from math import sqrt
from __builtin__ import pow

def heuristic(current, end):
    x = pow((current.point.x - end.point.x), 2)
    y = pow((current.point.y - end.point.y), 2)
    h = sqrt(x+y)
    return h

#takes in Start and End AStar nodes.
def AStar_search(start, end):
    #Set Start and End values
    start.g = 0
    start.h = heuristic(start, end)
    
    #FrontierSet is the set of nodes to be opened, i.e. Frontier
    FrontierSet = set()
    #ExpandedSet is the set of nodes already expanded
    ExpandedSet = set()
    
    #Add Start to frontier
    FrontierSet.add(start)
    
    while FrontierSet:
        PublishGridCells(pub_frontier, FrontierSet)
        PublishGridCells(pub_explored, ExpandedSet)
        PublishGridCells(pub_start, [start])
        PublishGridCells(pub_end, [end])
        
        #find the node in FrontierSet with the minimum heuristic value
        current = min(FrontierSet, key=lambda o:o.g + o.h)
        
        #If the goal is being expanded
        if current.poseEqual(end):
            #Construct path
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
                path.append(current)
            
            #Return path (less one garbage node that is appended)
            return path[::-1]
        
        #Else, move node from frontier to explored
        FrontierSet.remove(current)
        if not (current.poseEqual(start) and current.poseEqual(end)):
            ExpandedSet.add(current)
        
        #Check for possible 8-directional moves
        repeatedNode_flag = False
        somethingWasUpdatedFlag = False
        for node in WhereToGo(current):
            #Ignore if node is expanded
            for expanded in ExpandedSet:
                if node.poseEqual(expanded):
                    new_g = current.g + move_cost(current,node)
                    if node.g > new_g:
                        node.g = new_g
                        node.h = heuristic(node, end)
                        node.parent = current
                    repeatedNode_flag = True
                    somethingWasUpdatedFlag = True
                    break   
            
            #Try to update cost of traveling to node if already exists
            for frontier in FrontierSet:
                if node.poseEqual(frontier) and repeatedNode_flag == False:
                    new_g = current.g + move_cost(current,node)
                    if node.g > new_g:
                        node.g = new_g
                        node.h = heuristic(node, end)
                        node.parent = current
                        FrontierSet.remove(frontier)
                        FrontierSet.add(node)
                    somethingWasUpdatedFlag = True
                    break
                if somethingWasUpdatedFlag == True:
                    break
            #Add to frontier and update costs and heuristic values
            if somethingWasUpdatedFlag == False:
                node.g = current.g + move_cost(current, node)
                node.h = heuristic(node, end)
                node.parent = current
                FrontierSet.add(node)
            repeatedNode_flag = False
            somethingWasUpdatedFlag = False
    return None


def map_function(msg):
	global map_data
	map_data = msg.data

def getMapIndex(node):
    return int(round(((((node.point.y +3) * 5) *37) + ((node.point.x +3) * 5)),0))

#Takes in current node, returns list of possible directional movements
def WhereToGo(node):
    possibleNodes = []
    
    direction = Direction()

    #Hacky code begins
    North = AStarNode(round(node.point.x,1), round(node.point.y+0.2,1))
    North.g = node.g + 0.2
    North.step_direction = direction.n
    
    NorthEast = AStarNode(round(node.point.x+0.2,1), round(node.point.y+0.2,1))
    NorthEast.g = node.g + 0.29
    NorthEast.step_direction = direction.ne
    
    East = AStarNode(round(node.point.x+0.2,1), round(node.point.y, 1))
    East.g = node.g + 0.2
    East.step_direction = direction.e
    
    SouthEast = AStarNode(round(node.point.x+0.2,1), round(node.point.y-0.2,1))
    SouthEast.g = node.g + 0.29
    SouthEast.step_direction = direction.se
    
    South = AStarNode(round(node.point.x, 1), round(node.point.y-0.2,1))
    South.g = node.g + 0.2
    South.step_direction = direction.s
    
    SouthWest = AStarNode(round(node.point.x-0.2,1), round(node.point.y-0.2,1))
    SouthWest.g = node.g + 0.29
    SouthWest.step_direction = direction.sw
    
    West = AStarNode(round(node.point.x-0.2,1), round(node.point.y, 1))
    West.g = node.g + 0.2
    West.step_direction = direction.w
    
    NorthWest = AStarNode(round(node.point.x-0.2,1), round(node.point.y+0.2,1))
    NorthWest.g = node.g + 0.29
    NorthWest.step_direction = direction.nw
    
    if map_data[getMapIndex(North)] != 100:
        possibleNodes.append(North)
    if map_data[getMapIndex(NorthEast)] != 100:
        possibleNodes.append(NorthEast)
    if map_data[getMapIndex(East)] != 100:
        possibleNodes.append(East)
    if map_data[getMapIndex(SouthEast)] != 100:
        possibleNodes.append(SouthEast)
    if map_data[getMapIndex(South)] != 100:
        possibleNodes.append(South)
    if map_data[getMapIndex(SouthWest)] != 100:
        possibleNodes.append(SouthWest)
    if map_data[getMapIndex(West)] != 100:
        possibleNodes.append(West)
    if map_data[getMapIndex(NorthWest)] != 100:
        possibleNodes.append(NorthWest)

    #Hacky code begins
    return possibleNodes

def move_cost(node, next):
    diagonal = round(abs(node.point.x - next.point.x),1) == .2 
    diagonal = diagonal and round(abs(node.point.y - next.point.y),1) == .2
    if diagonal:
        return 0.29
    else:
        return 0.2

class AStarNode():
    
    def __init__(self, x, y):
        direction = Direction()
        self.point = Point()
        self.point.x, self.point.y = x, y
        self.g = 0
        self.h = 0
        self.parent = None
        self.step_direction = direction.start
        
    def poseEqual(self, node):
        return node.point.x == self.point.x and node.point.y == self.point.y 
    

class Direction():
    def __init__(self):
        self.n = 1
        self.s = 2
        self.e = 3
        self.w = 4
        self.ne = 5
        self.se = 6
        self.nw = 7
        self.sw = 8
        self.start = 0
        
def getWaypoints(path):
    waypoints = []
    direction = Direction()
    previous = direction.start
    
    for node in path:
        if(node.step_direction != previous):
            waypoints.append(node.parent)
            previous = node.step_direction
        else:
            continue
    
    waypoints.append(start)
    waypoints.append(end)
        
    return waypoints    

#Publish Explored Cells function
def PublishGridCells(publisher, nodes):
    #Initialize gridcell
    gridcells = GridCells()
    gridcells.header.frame_id = 'map'
    gridcells.cell_width = Map_Cell_Width
    gridcells.cell_height = Map_Cell_Height

    #Iterate through list of nodes
    for node in nodes: 
        point = Point()
        point.x = node.point.x
        point.y = node.point.y
        #Ensure z axis is 0 (2d Map)
        point.z = node.point.z = 0
        gridcells.cells.append(point)        
    publisher.publish(gridcells)
    rospy.sleep(rospy.Duration(0.05,0))

def run_Astar():
    
    PublishGridCells(pub_start, [start])
    PublishGridCells(pub_end, [end])
    PublishGridCells(pub_path, [])
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    
    path = AStar_search(start, end)
    
    print "Displaying Path"
    PublishGridCells(pub_path, path)
    
    blah = raw_input("Press enter to display ")
    PublishGridCells(pub_path, [])
    
    print "Showing Waypoints"
    waypoints = getWaypoints(path)
    rospy.sleep(rospy.Duration(1,0))
    PublishGridCells(pub_path, waypoints)

#####################################3
# set and print initialpose
def set_initial_pose (msg):
    
    global start_pos_x
    global start_pos_y
    global start_pos_flag
    #global start_pos_z
    #global start_orient_x
    #global start_orient_y
    #global start_orient_z
    #global start_w
    
    start_pos_x = msg.pose.pose.position.x
    start_pos_y = msg.pose.pose.position.y
    
    #set initial pose values
    if msg.pose.pose.position.x % .2 < .1:
    	start_pos_x = start_pos_x - (msg.pose.pose.position.x % .2)
    else:
    	start_pos_x = start_pos_x - (msg.pose.pose.position.x % .2) + .2
    
    if msg.pose.pose.position.y % .2 < .1:
    	start_pos_y = start_pos_y - (msg.pose.pose.position.y % .2)
    else:
    	start_pos_y = start_pos_y - (msg.pose.pose.position.y % .2) + .2
    
    #if msg.pose.pose.position.x % .2 < .1:
    #	start_pos_x = (msg.pose.pose.position.x / .2)
    #else:
    #	start_pos_x = (msg.pose.pose.position.x / .2) + .2
    
    #we have not checked scaling or anything for these:
    #start_orient_x = msg.pose.pose.orientation.x
    #start_orient_y = msg.pose.pose.orientation.y
    #start_orient_z = msg.pose.pose.orientation.z
    #start_w = msg.pose.pose.orientation.w
    
    #print initial pose values
    start.point.x = start_pos_x
    start.point.y = start_pos_y
    print ""
    print "Initial Pose Values:"
    print "start_pos_x = ", start.point.x
    print "start_pos_y = ", start.point.y
    #print "start_pos_z = ", start_pos_z
    
   
    
    start_pos_flag = True
    #we have not checked scaling or anything for these:
    #print "start_orient_x = ", start_orient_x
    #print "start_orient_y = ", start_orient_y
    #print "start_orient_z = ", start_orient_z
    #print "start_w = ", start_w

# set and print goalpose
def set_goal_pose (msg):
    
    global end_pos_flag
    end_pos_x = msg.pose.position.x
    end_pos_y = msg.pose.position.y
    #set initial pose values
    if msg.pose.position.x % .2 < .1:
        end_pos_x = end_pos_x - (msg.pose.position.x % .2)
    else:
        end_pos_x = end_pos_x - (msg.pose.position.x % .2) + .2
    
    if msg.pose.position.y % .2 < .1:
        end_pos_y = end_pos_y - (msg.pose.position.y % .2)
    else:
        end_pos_y = end_pos_y - (msg.pose.position.y % .2) + .2
        
        
    end.point.x = end_pos_x
    end.point.y = end_pos_y
    
    print "end_pos_x = ", end.point.x
    print "end_pos_y = ", end.point.y 
    
    end_pos_flag = True
    

#######################################
# This is the program's main function
if __name__ == '__main__':
    
    # Change this node name to include your username
    rospy.init_node('rbansal_vcunha_dbourque_Lab3Node')
    
    
    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global publisher
    global pub_start
    global pub_end
    global pub_explored
    global pose
    global odom_tf
    global odom_list
    global Map_Cell_Width
    global Map_Cell_Height
    global map_data
    
    global start_pos_flag
    global end_pos_flag
    
   
    #global start_pos_z
    #global start_orient_x
    #global start_orient_y
    #global start_orient_z
    #globalstart_w
    global goal_pos_x
    global goal_pos_y
    #global goal_pos_z
    #global goal_orient_x
    #global goal_orient_y
    #global goal_orient_z
    #global goal_w
    
    Map_Cell_Width = 0.2
    Map_Cell_Height = 0.2
        
    #Publishers: 
    pub_start    = rospy.Publisher('/start', GridCells) # Publisher for start Point
    pub_end      = rospy.Publisher('/end'  , GridCells) # Publisher for End Point
    pub_path     = rospy.Publisher('/path' , GridCells) # Publisher for Final Path
    pub_explored = rospy.Publisher('/explored', GridCells) # Publisher explored GridCells
    pub_frontier = rospy.Publisher('/frontier', GridCells) # Publisher explored GridCells
       
    #Subscribers:
    sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, set_initial_pose, queue_size=1)
    sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, set_goal_pose, queue_size=1)  
    sub = rospy.Subscriber('/map', OccupancyGrid, map_function, queue_size=1)
    
    start_pos_flag = False
    end_pos_flag = False
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    
    print "Starting Lab 3"
    start = AStarNode(-1,-1.8)
    end = AStarNode(1,1.8);
    while(1):
        if(start_pos_flag == True and end_pos_flag == True):
            print 'running Astar'
            run_Astar();
            start_pos_flag = False
            end_pos_flag = False
            
    print "Lab Complete"


    

        
    
