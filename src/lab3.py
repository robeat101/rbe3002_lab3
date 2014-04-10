#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped
from numpy import ma

def heuristic(current, end):
    x = (current.point.x - end.point.x)**2
    y = (current.point.y - end.point.y)**2
    return round((x + y)**0.5, 2)

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
    add_to_FrontierSet_flag = 0 #initialize add to FrontierSet flag
    add_to_ExpandedSet_flag = 0 #initialize add to ExpandedSet flag
    while FrontierSet:
        print "Frontier Set:"
        PublishGridCells(pub_frontier, FrontierSet)
        print "Expanded Set:"
        PublishGridCells(pub_explored, ExpandedSet)
        print "start:"
        PublishGridCells(pub_start, [start])
        print "end:"
        PublishGridCells(pub_end, [end])
        rospy.sleep(rospy.Duration(0.1, 0))

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
        
        #Else, move node from frontier to explored set
        FrontierSet.remove(current)
        if not ExpandedSet:
            ExpandedSet.add(current)
            print "ADDED TO EMPTY EXPANDEDSET"
        else:
            add_to_ExpandedSet_flag = 1
            for node in ExpandedSet:
                if ((current.point.x != node.point.x) & (current.point.y != node.point.y)):
                    #skip if has been set to 0 already
                    if add_to_ExpandedSet_flag == 0:
                        continue
                    #set to 1 since new expanded node
                    else:
                        add_to_ExpandedSet_flag = 1
                #set to 0 if not a new expanded 
                else:
                    add_to_ExpandedSet_flag = 0
        if add_to_ExpandedSet_flag == 1:
            print "ADDED TO EXPANDEDSET"
            ExpandedSet.add(current)
            add_to_ExpandedSet_flag = 0
        
        #Check for possible 8-directional moves
        for node in WhereToGo(current):
            wtg_node = node
            add_to_FrontierSet_flag = 1
            in_ExpandedSet_flag = 1
            #Ignore if node is expanded
            for node in ExpandedSet:
                if ((wtg_node.point.x == node.point.x) & (wtg_node.point.y == node.point.y)):
                    print "NODE IN EXPANDEDSET"
                    in_ExpandedSet_flag = 0
            if in_ExpandedSet_flag == 1:
                "ARE WE HERE?"
                # if empty frontier (at beginning especially)
                if not FrontierSet:
                    print "frontier set is empty"
                    FrontierSet.add(wtg_node)
                else:
                    #Try to update cost of traveling to node if already exists
                    for node in FrontierSet:
                        if ((wtg_node.point.x == node.point.x) & (wtg_node.point.y == node.point.y)):
                            new_g = current.g + move_cost(current,wtg_node)
                            if node.g > new_g:
                                node.g = new_g
                                node.parent = current
                                add_to_FrontierSet_flag = 0
                        #Add to frontier and update costs and heuristic values
                        else:
                            node.g = current.g + move_cost(current, wtg_node)
                            node.h = heuristic(wtg_node, end)
                            node.parent = current
                            if add_to_FrontierSet_flag == 0:
                                continue
                if add_to_FrontierSet_flag == 1:
                    FrontierSet.add(wtg_node)
                    print "NODE ADDED TO FRONTIERSET"
			
    return None

def map_function(msg):
	global map_data
	map_data = msg.data

def getMapIndex(node):
    return int(round(((((node.point.y +3) * 5) *37) + ((node.point.x +3) * 5)),0))

#Takes in current node, returns list of possible directional movements
def WhereToGo(node):
    possibleNodes = []

    #Hacky code begins
    North = AStarNode(round(node.point.x,1), round(node.point.y+0.2,1))
    NorthEast = AStarNode(round(node.point.x+0.2,1), round(node.point.y+0.2,1))
    East = AStarNode(round(node.point.x+0.2,1), node.point.y)
    SouthEast = AStarNode(round(node.point.x+0.2,1), round(node.point.y-0.2,1))
    South = AStarNode(node.point.x, round(node.point.y-0.2,1))
    SouthWest = AStarNode(round(node.point.x-0.2,1), round(node.point.y-0.2,1))
    West = AStarNode(round(node.point.x-0.2,1), node.point.y)
    NorthWest = AStarNode(round(node.point.x-0.2,1), round(node.point.y+0.2,1))
	
    print(North)

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

    return possibleNodes

def move_cost(node, next):
    diagonal = abs(node.point.x - next.point.x) == .2 and abs(node.point.y - next.point.y) == .2
    return round( (.2*.2 + .2 *.2)**0.5 if diagonal else .2 , 2)

class AStarNode():
    
    def __init__(self, x, y):
        self.point = Point()
        self.point.x, self.point.y = x, y
        self.g = 0
        self.h = 0
        self.parent = None
        
    def poseEqual(self, node):
        return node.point.x == self.point.x and node.point.y == self.point.y 
   
#Publish Explored Cells function
def PublishGridCells(publisher, nodes):
    #Initialize gridcell
    gridcells = GridCells()
    gridcells.header.frame_id = 'map'
    gridcells.cell_width = Map_Cell_Width
    gridcells.cell_height = Map_Cell_Height

    #Iterate through list of nodes
    for node in nodes: 
        print 'Iterating ' + str(node.point.x) + str(node.point.y)
        point = Point()
        point.x = node.point.x
        point.y = node.point.y
        #Ensure z axis is 0 (2d Map)
        point.z = node.point.z = 0
        gridcells.cells.append(point)
        
    print publisher
    publisher.publish(gridcells)
    rospy.sleep(rospy.Duration(.1, 0))
    

#####################################3
# set and print initialpose
def set_initial_pose (msg):
	
	global start_pos_x
	global start_pos_y
	#global start_pos_z
	#global start_orient_x
	#global start_orient_y
	#global start_orient_z
	#global start_w

	#set initial pose values
	if msg.pose.pose.position.x % .2 < .1:
		start_pos_x = (msg.pose.pose.position.x / .2)
	else:
		start_pos_x = (msg.pose.pose.position.x / .2) + .2

	if msg.pose.pose.position.y % .2 < .1:
		start_pos_y = (msg.pose.pose.position.y / .2)
	else:
		start_pos_y = (msg.pose.pose.position.y / .2) + .2

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
	print ""
	print "Initial Pose Values:"
	print "start_pos_x = ", start_pos_x
	print "start_pos_y = ", start_pos_y
	#print "start_pos_z = ", start_pos_z

	#we have not checked scaling or anything for these:
	#print "start_orient_x = ", start_orient_x
	#print "start_orient_y = ", start_orient_y
	#print "start_orient_z = ", start_orient_z
	#print "start_w = ", start_w

# set and print goalpose
def set_goal_pose (msg):

	global goal_pos_x
	global goal_pos_y
	#global goal_pos_z
	#global goal_orient_x
	#global goal_orient_y
	#global goal_orient_z
	#global goal_w

	#set goal pose values
	goal_pos_x = msg.pose.position.x
	goal_pos_y = msg.pose.position.y
	#goal_pos_z = msg.pose.position.z
	
	#we have not checked scaling or anything for these:
	#goal_orient_x = msg.pose.orientation.x
	#goal_orient_y = msg.pose.orientation.y
	#goal_orient_z = msg.pose.orientation.z
	#goal_w = msg.pose.orientation.w
	
	#print goal pose values
	print ""
	print "Goal Pose Values:"
	print "goal_pos_x = ", goal_pos_x
	print "goal_pos_y = ", goal_pos_y
	#print "goal_pos_z = ", goal_pos_z

	#we have not checked scaling or anything for these:
	#print "goal_orient_x = ", goal_orient_x
	#print "goal_orient_y = ", goal_orient_y
	#print "goal_orient_z = ", goal_orient_z
	#print "goal_w = ", goal_w



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
    
    global start_pos_x
    global start_pos_y
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
    pub_explored = rospy.Publisher('/explored', GridCells) # Publisher explored GridCells
    pub_frontier = rospy.Publisher('/frontier', GridCells) # Publisher explored GridCells
    pub_start    = rospy.Publisher('/start', GridCells) # Publisher for start Point
    pub_end      = rospy.Publisher('/end'  , GridCells) # Publisher for End Point
        
    #Subscribers:
    sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, set_initial_pose, queue_size=1)
    sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, set_goal_pose, queue_size=1)  
    sub = rospy.Subscriber('/map', OccupancyGrid, map_function, queue_size=1)
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    
    print "Starting Lab 3"
    # Hardcoded start and end points: 
    start  = AStarNode(-1, -1)
    end    = AStarNode(1.4, -1)
    PublishGridCells(pub_start, [start])
    PublishGridCells(pub_end, [end])
    
    AStar_search(start, end)
    
    
    print "Lab 3 complete!"
    rospy.spin()
    
    
