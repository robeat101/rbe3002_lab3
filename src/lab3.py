#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from numpy import ma

def heuristic(current_point, end_point):
    x = (current_point.x - end_point.x)**2
    y = (current_point.y - end_point.y)**2
    return (x + y)**0.5


def run_Astar(start, end):
    pass

def search(start, end):
    openset = set()
    closedset = set()
    current = start
    openset.add(current)
    while openset:
        current = min(openset, key=lambda o:o.g + o.h)
        if current == end:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
                path.append(current)
                return path[::-1]
        openset.remove(current)
        closedset.add(current)
        for node in self.graph[current]:
            if node in closedset:
                continue
            if node in openset:
                new_g = current.g + current.move_cost(node)
                if node.g > new_g:
                    node.g = new_g
                    node.parent = current
            else:
                node.g = current.g + current.move_cost(node)
                node.h = self.heuristic(node, start, end)
                node.parent = current
                openset.add(node)
    return None
 
class AStarNode(object):
    def __init__(self):
        self.g = 0
        self.h = 0
        self.parent = None

#Publish Explored Cells function
def PublishGridCells(publisher, points):
    
    #Initialize gridcell
    gridcells = GridCells()
    gridcells.header.frame_id = 'map'
    gridcells.cell_width = Map_Cell_Width
    gridcells.cell_height = Map_Cell_Height
    
    #Iterate through list of points
    for point in points: 
        #Ensure z axis is 0 (2d Map)
        point.z = 0
        gridcells.cells.append(point)
    
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

	# Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(1, 0))



	print "Starting Lab 3"

	# Hardcoded start and end points: 
	start  = Point(-1, -1, 0)
	end    = Point(1.4, 1.4, 0)
	PublishGridCells(pub_start, [start])
	PublishGridCells(pub_end, [end])

	run_Astar(start, end)

	print "Lab 3 complete!"
	rospy.spin()

