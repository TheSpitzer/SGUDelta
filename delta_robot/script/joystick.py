#! /usr/bin/env python
import roslib; roslib.load_manifest('delta_robot')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import rospy
import math
from delta_construct import create_delta_simulation, set_initial_delta_pose, move_simulation_to_point
from delta_kinematics import *
from transformations import *
import numpy as np
import random
import csv

#Data Typing the variable
global pub1
pub1 = rospy.Publisher('/joint1_controller/command', Float64, queue_size = 1)
global pub2
pub2 = rospy.Publisher('/joint2_controller/command', Float64, queue_size = 1)
global pub3
pub3 = rospy.Publisher('/joint3_controller/command', Float64, queue_size = 1)

def move_dynamixels(alpha, beta, theta):
    #Publish message to actuate the servo
    global pub1
    pub1.publish(Float64(math.radians(alpha)))
    global pub2
    pub2.publish(Float64(math.radians(beta)))
    global pub3
    pub3.publish(Float64(math.radians(theta)))

def JointArrayCb(data):
    #Calculate equivalent positions from the image to the robot
    myx = data.linear.x - 0.08
    myy = data.linear.y - 0.08
    myz = data.linear.z - 0.08
    angles = delta_calcInverse(myx, myy, myz)
    print str(angles[1]) + ", "+ str(angles[2]) + ", "+ str(angles[3])

    
    # Calling function to move_dynamixels( alpha, beta, theta)
    move_dynamixels(angles[1], angles[2], angles[3])

#Sourcing node name
topic = 'joystick_controller'
rospy.init_node('delta_robot')
rospy.Subscriber("/coordinate", Twist , JointArrayCb)
rospy.spin()
