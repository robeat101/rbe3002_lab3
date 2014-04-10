#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
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

    Map_Cell_Width = 0.2
    Map_Cell_Height = 0.2
    
    
    #Publishers: 
    pub_explored = rospy.Publisher('/explored', GridCells) # Publisher explored GridCells
    pub_start    = rospy.Publisher('/start', GridCells) # Publisher for start Point
    pub_end      = rospy.Publisher('/end'  , GridCells) # Publisher for End Point

    
    
    
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

