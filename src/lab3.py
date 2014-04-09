#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from numpy import ma


def  run_Astar(start, end):
    pass #

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

