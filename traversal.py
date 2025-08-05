#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

range_max = 8.0
regions = {'bright': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'bleft': 0,}
prev_regions = {'bright': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'bleft': 0,}
pose = [0,0,0]
prev_pose = [0,0,0]
first_odom = 1
first_laser = 1
velocity_msg = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def control_loop():

    global prev_pose
    global prev_regions
    global velocity_msg
    global pub

    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    
    alt_dir = 1

    turn('left')

    #Almost the same as the "tvs" function which is defined later, except one if condition
    while not rospy.is_shutdown():

        while(regions.get('bright') > 1.25 and not rospy.is_shutdown()):
            tilt_error = prev_pose[2] - pose[2]
            velocity_msg.linear.x = 0.5
            velocity_msg.angular.z = tilt_error*1
            pub.publish(velocity_msg)
            prev_regions = regions
        
        while(regions.get('bright') < 1.25 and not rospy.is_shutdown()):
            tilt_error = prev_regions.get('bright') - regions.get('bright')
            velocity_msg.linear.x = 0.5
            velocity_msg.angular.z = tilt_error*10
            pub.publish(velocity_msg)
            prev_regions = regions

        #This if condition uses the laser scanner data to check if there is any other row, thus allowing the bot to reach to the end before starting the main algorithm. This allows the bot to traverse in greenhouses with more than just 2 rows
        if(regions.get('fright') > 1.5 or regions.get('front') < 2.5):
            
            #To make the bot traverse some extra distance, with a hardcoded number of iterations, which was experimentally found for smooth performance.
            extra_dist(20000)
            break
    

    turn('right')

    tvs('right')
    
    while not rospy.is_shutdown():

        # To keep alternating between values being sensed and published, so the bot can traverse in the required pattern
        if(alt_dir == 1):
            dir = 'right'
        elif(alt_dir == -1):
            dir = 'left'
        turn(dir)
        th(dir)
        # This if condition allows the bot to check if there is another row. If it senses another row, it will traverse using tvm, else it will use tvs since it's the last row, and stop the execution. This allows the bot to traverse in greenhouses with more than just 2 rows
        if(regions.get('f' + dir) < 1.5 and regions.get('front') > 2.5):
            turn(dir)
            tvm()
        else:
            turn(dir)
            tvs(dir)
            break
                
        alt_dir = -(alt_dir)

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    #print("Controller message pushed at {}".format(rospy.get_time()))
    rate.sleep()

def odom_callback(data):
    global pose
    global first_odom
    global prev_pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

    # To avoid jump in yaw values from 3.14 to -3.14
    if(pose[2] > -3.15 and pose[2] < -2.5):
        pose[2] = 2*math.pi + pose[2]

    #To store the initial pose values
    if(first_odom == 1):
        prev_pose = pose
        first_odom = 0

def laser_callback(msg):
    global regions
    global prev_regions
    global first_laser
    regions = {
        'bright': min(min(msg.ranges[0:10]), range_max),
        'fright': min(min(msg.ranges[60:300]), range_max),
        'front': min(min(msg.ranges[330:390]), range_max),
        'fleft': min(min(msg.ranges[520:660]), range_max),
        'bleft': min(min(msg.ranges[710:720]), range_max),
    }

    #To store the initial laser scan values
    if(first_laser == 1):
        prev_regions = regions
        first_laser = 0

#Function to make the bot turn 90 degrees in a given direction
def turn(direction):

    global pose
    global prev_pose
    global regions
    global prev_regions
    global pub
    global velocity_msg

    while not rospy.is_shutdown():

        #To change the sign of the angular velocity being published, corresponding to the direction given
        if(direction == 'left'):
            change = 1
        elif(direction == 'right'):
            change = -1

        #Publishing non zero angular velocity till change in yaw value is about pi/2 (1.5 is used for better accuracy (found experimentally))
        if((pose[2] - prev_pose[2])*change < 1.5):

            velocity_msg.angular.z = change

        #Publishing zero angular velocity after the 90 degree turn, and re caliberating prev_pose and prev_regions
        else:

            velocity_msg.angular.z = 0
            prev_pose = pose
            prev_regions = regions
            pub.publish(velocity_msg)
            break
        
        pub.publish(velocity_msg)

#Function to make the bot TRAVERSE VERTICALLY at a SIDE, given the side at which the row is present
def tvs(direction):

    global pose
    global prev_pose
    global regions
    global prev_regions
    global velocity_msg
    global pub

    while not rospy.is_shutdown():

        #To change the sign of the angular velocity being published, corresponding to the direction given
        if(direction == 'right'):
            error_fix = 1
            opp = 'left'
        elif(direction == 'left'):
            error_fix = -1
            opp = 'right'

        #Traversing till 'fright' or 'fleft' senses the wall in the corresponding direction
        if(regions.get('f'+ direction) < 2):
            #tilt error is calculated using laser data, such that the bot stays 3 times closer to the row than the greenhouse wall, and the same is published as the angular velocity for correction, multiplied with an appropriate gain constant
            tilt_error = regions.get('f' + opp) - 3*regions.get('f' + direction)
            velocity_msg.linear.x = 1
            velocity_msg.angular.z = (tilt_error*10)*error_fix
            prev_regions = regions
                
        else:

            #Traversing till 'bright' or 'bleft' senses the wall in the corresponding direction
            if(regions.get('b' + direction) < 1.25):
                tilt_error = regions.get('b' + opp) - 3*regions.get('b' + direction)
                velocity_msg.linear.x = 1
                velocity_msg.angular.z = (tilt_error*10)*error_fix
                prev_regions = regions

            else:

                #To make the bot traverse some extra distance, with a hardcoded number of iterations, which was experimentally found for smooth performance.
                extra_dist(30000)
                break
        
        pub.publish(velocity_msg)

#Function to make the bot TRAVERSE HORIZONTALLY when switching between lanes
def th(direction):

    global pose
    global prev_pose
    global regions
    global prev_regions
    global velocity_msg
    global pub

    #To change the sign of the angular velocity being published, corresponding to the direction given
    if(direction == 'right'):
        error_fix = 1
    elif(direction == 'left'):
        error_fix = -1

    #Values read by the ends of the laser scan "'b' + direction" will be high before the bot comes across the horizontal wall of the row

    #To make the bot traverse horizontally till it reaches the horizontal wall of the row, using difference of previous yaw and present yaw to calculate error in direction.
    while(regions.get('b' + direction) > 1.25 and not rospy.is_shutdown()):
        #tilt error is calculated by comparing previous pose with current pose, and the same is published as the angular velocity for correction, multiplied with an appropriate gain constant
        tilt_error = prev_pose[2] - pose[2]
        velocity_msg.linear.x = 0.5
        velocity_msg.angular.z = tilt_error*1
        pub.publish(velocity_msg)
        prev_regions = regions
    
    #To make the bot traverse horizontally till it stops sensing the horizontal wall of the row, using difference of distances from the wall to calculate error in direction.
    while(regions.get('b' + direction) < 1.25 and not rospy.is_shutdown()):
        #tilt error is calculated by comparing previous laser data with current laser data, and the same is published as the angular velocity for correction, multiplied with an appropriate gain constant
        tilt_error = prev_regions.get('b' + direction) - regions.get('b' + direction)
        velocity_msg.linear.x = 0.5
        velocity_msg.angular.z = (tilt_error*10)*error_fix
        pub.publish(velocity_msg)
        prev_regions = regions
    
    #To make the bot traverse some extra distance, with a hardcoded number of iterations, which was experimentally found for smooth performance.
    extra_dist(20000)

def tvm():

    global pose
    global prev_pose
    global regions
    global prev_regions
    global velocity_msg
    global pub

    while not rospy.is_shutdown():

        if(regions.get('fright') < 1.5 or regions.get('fleft') < 1.5):
            #tilt error is calculated using laser data, such that the bot traverses at the center of both the rows, and the same is published as the angular velocity for correction, multiplied with an appropriate gain constant
            tilt_error = regions.get('fleft') - regions.get('fright')
            velocity_msg.linear.x = 1
            velocity_msg.angular.z = tilt_error*10
            prev_regions = regions
        
        else:

            if(regions.get('bright') < 1.25 or regions.get('bleft') < 1.25):
                tilt_error = regions.get('bleft') - regions.get('bright')
                velocity_msg.linear.x = 1
                velocity_msg.angular.z = tilt_error*10
                prev_regions = regions

            else:

                #To make the bot traverse some extra distance, with a hardcoded number of iterations, which was experimentally found for smooth performance.
                extra_dist(30000)
                break
        
        pub.publish(velocity_msg)

#Function to make the bot move some extra distance, with an arguement deciding the number of the for loop iterations based on the requirement
def extra_dist(iterations):
    global pose
    global prev_pose
    global velocity_msg
    global pub

    for i in range(iterations):
        #tilt error is calculated by comparing previous pose with current pose, and the same is published as the angular velocity for correction, multiplied with an appropriate gain constant
        tilt_error = prev_pose[2] - pose[2]
        velocity_msg.linear.x = 1
        velocity_msg.angular.z = tilt_error*1
        pub.publish(velocity_msg)

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
