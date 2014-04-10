#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from numpy import ma

def heuristic(current, end):
    x = (current.point.x - end.point.x)**2
    y = (current.point.y - end.point.y)**2
    return (x + y)**0.5

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
        PublishGridCells(pub_explored, FrontierSet)
        #find the node in FrontierSet with the minimum heuristic value
        current = min(FrontierSet, key=lambda o:o.g + o.h)
        
        #If the goal is being expanded
        if current == end:
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
        ExpandedSet.add(current)
        
        #Check for possible 8-directional moves
        for node in WhereToGo(current):
            #Ignore if node is expanded
            if node in ExpandedSet:
                continue
            #Try to update cost of traveling to node if already exists
            if node in FrontierSet:
                new_g = current.g + move_cost(current,node)
                if node.g > new_g:
                    node.g = new_g
                    node.parent = current
            #Add to frontier and update costs and heuristic values
            else:
                node.g = current.g + move_cost(current, node)
                node.h = heuristic(node, end)
                node.parent = current
                FrontierSet.add(node)
    return None

def WhereToGo(node):
    newNode = AStarNode(node.point.x+0.2, node.point.y)
    return [newNode]

def move_cost(node, next):
    diagonal = abs(node.point.x - next.point.x) == .2 and abs(node.point.y - next.point.y) == .2
    return (.2*.2 + .2 *.2)**0.5 if diagonal else .2

class AStarNode():
    
    def __init__(self, x, y):
        self.point = Point()
        self.point.x, self.point.y = x, y
        self.g = 0
        self.h = 0
        self.parent = None
   
#Publish Explored Cells function
def PublishGridCells(publisher, nodes):
    
    #Initialize gridcell
    gridcells = GridCells()
    gridcells.header.frame_id = 'map'
    gridcells.cell_width = Map_Cell_Width
    gridcells.cell_height = Map_Cell_Height
    
    #Iterate through list of nodes
    for node in nodes: 
        #Ensure z axis is 0 (2d Map)
        node.point.z = 0
        gridcells.cells.append(node.point)
    
    publisher.publish(gridcells)
    

#####################################3
# print initialpose
def set_initial_pose (msg):
	
	global start_pos_x
	global start_pos_y
	#global start_pos_z
	#global start_orient_x
	#global start_orient_y
	#global start_orient_z
	#global start_w

	#set initial pose values
	start_pos_x = msg.pose.pose.position.x
	start_pos_y = msg.pose.pose.position.y
	start_pos_z = msg.pose.pose.position.z
	start_orient_x = msg.pose.pose.orientation.x
	start_orient_y = msg.pose.pose.orientation.y
	start_orient_z = msg.pose.pose.orientation.z
	start_w = msg.pose.pose.orientation.w
	
	#print initial pose values
	print ""
	print "Initial Values:"
	print "start_pos_x = ", start_pos_x
	print "start_pos_y = ", start_pos_y
	#print "start_pos_z = ", start_pos_z
	#print "start_orient_x = ", start_orient_x
	#print "start_orient_y = ", start_orient_y
	#print "start_orient_z = ", start_orient_z
	#print "start_w = ", start_w


#######################################
# This is the program's main function
if __name__ == '__main__':

    # Change this node name to include your username
    rospy.init_node('rbansal_vcunha_dbourque_Lab3Node')
    
    
    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub_explored
    global pose
    global odom_tf
    global odom_list
    global Map_Cell_Width
    global Map_Cell_Height
    
    global start_pos_x
    global start_pos_y
    #global start_pos_z
    #global start_orient_x
    #global start_orient_y
    #global start_orient_z
    #global start_w
    
    Map_Cell_Width = 0.2
    Map_Cell_Height = 0.2
    
    
    #Publishers: 
    pub_explored = rospy.Publisher('/explored', GridCells) # Publisher explored GridCells
    pub_start    = rospy.Publisher('/start', GridCells) # Publisher for start Point
    pub_end      = rospy.Publisher('/end'  , GridCells) # Publisher for End Point
    
    #Subscribers:
    sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, set_initial_pose, queue_size=1)
    print "Starting Lab 3"
    
    # Hardcoded start and end points: 
    start  = AStarNode(-1, -1)
    end    = AStarNode(1.4, -1)
    PublishGridCells(pub_start, [start])
    PublishGridCells(pub_end, [end])
    
    AStar_search(start, end)
    
    
    print "Lab 3 complete!"
    rospy.spin()

