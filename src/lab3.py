#!/usr/bin/env python

import rospy, tf
from kobuki_msgs.msg import BumperEvent
# Add additional imports for each of the message types used






#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    pass  # Delete this 'pass' once implemented






#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    pass  # Delete this 'pass' once implemented



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    pass  # Delete this 'pass' once implemented


    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    pass  # Delete this 'pass' once implemented



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented





#Odometry Callback function.
def read_odometry(msg):
    pass  # Delete this 'pass' once implemented
  


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        pass  # Delete this 'pass' once implemented



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented









# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global pose
    global odom_tf
    global odom_list

    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    #pub = rospy.Publisher('...', ...) # Publisher for commanding robot motion
    #sub = rospy.Subscriber('...', ..., read_odometry, queue_size=1) # Callback function to read in robot Odometry messages

    #bumper_sub = rospy.Subscriber('...', ..., readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))



    print "Starting Lab 2"

    # Make the robot do stuff...

    print "Lab 2 complete!"


