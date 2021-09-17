  
#! /usr/bin/env python

## @package rt2_assignment1
#  \file go_to_point.py
#  \brief Node implementing the go_to_point behaviour
#  \author Carmine Recchiuto, Ermanno Girardo
#  \version 1.2
#  \date September 2021
#
#  \details
#
# Subscribes to: <BR>
#	/odom
#	/slider
#
#  Publishes to: <BR>
#	/cmd_vel
#
#  Action server: <BR>
# 	/go_to_point
#
#  Action server used to reach the position
#  of a random target. In this script is implemented
#  the behaviour of an Finite State Machine in order to:
#  align the robot with the goal
#  move the robot in the direction of the goal
#  once the goal is reached the robot align its orientation 
#  with the goal's orientation
#  This node uses a publisher in order to publish the velocity
#  of the robot  on /cmd_vel topic and it uses a subscriber
#  in order to check the robot's position on /odom topic.
##

import rospy
import actionlib
import rt2_assignment1.msg
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
#from rt2_assignment1.srv import Position
import math



## robot state variables
## point used to store the actual position
position_ = Point()
feedback=rt2_assignment1.msg.PositionFeedback()
result=rt2_assignment1.msg.PositionResult()
## yaw angle set to 0 as default
yaw_ = 0
## position set to 0 as default
position_ = 0
##state set to 0 as default
state_ = 0
## publisher that publish to cmd_vel
pub_ = None

## parameters for control
yaw_precision_ = math.pi / 9  ## +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  ## +/- 2 degree allowed
## distance precision from goal allowed
dist_precision_ = 0.1
## angular proportional constant
kp_a = -3.0 
## linear proportional constant
kp_d = 0.2
## upper bound angular velocity
ub_a = 0.6
##lower bound angular velocity
lb_a = -0.5
## upper bound linear velociry
ub_d = 0.6

## action server
server=None


## function  callback of the subscriper of topic /slider
#
#  the slider modify the parameter for angular and linear velocity
#
#  /param msg_slider: of type twist is the current value of the slider
#  the slider modify the parameter for angular and linear velocity
##
def clbk_slider(msg_slider):
    global kp_a
    global kp_d;
    kp_d=msg_slider.linear.x
    kp_a=msg_slider.angular.z


## function callback of the subscriber of the topic /odom
#  it retrieves the x y theta coordinates from the Odom message.
#  /param msg: of type Odometry
##
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]



##  funcion used to change the state of the FSM
#
#   /param state(int): new state of the FSM
##
def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)



##  function used to normalize the angle
#
#   /param angle(float): angle to normalize
#   /return: angle normalized
##
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


##  function used to orient the robot in a desired direction
#   
#   The function is used  to orient the robot toward the goal
#  
#   /param des_pos(Point): desired position to compute the desired yaw
##
def fix_yaw(des_pos):
    global kp_a
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)



##  Once robot is aligned this function drive the robot toward the goal
#   
#   Set the linear and the angular velocity depending on the distance to the
#   goal pose.
#
#   /param des_pos(Point): desired x y position
##
def go_straight_ahead(des_pos):
    global kp_a, kp_d
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d
        if twist_msg.linear.x > ub_d:
           twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


##  Once the goal (x,y) is achieved the function 
#   orients the robot with the goal orientation
#   
#   /param des_yaw(float): desired yaw
## 
def fix_final_yaw(des_yaw):
    global kp_a
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        

##  Stop the robot
#   Set the linear and the angular velocity to 0
##
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


    

##  Function used to action server in order to allow
#   the robot to reach the correct target position
#   Set an appropriate behaviour depending on the
#   current robot state.
#   State machine keeps running until the goal is reached 
#   or the action is preempted: the goal is canceled
#
#   /param goal (PoseActionGoal): x,y,theta coordinates
#   of the goal pose
##
def go_to_point(goal):
    global server, state_
    ##instanciate the message for position result
    result=rt2_assignment1.msg.PositionResult()
    desired_position = Point()
    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta
    change_state(0)
    success=True
    while not rospy.is_shutdown():
        
        if server.is_preempt_requested():
            server.set_preempted()
            success = False
            done()
            break

        if state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            break
    

    if success:
    	   server.set_succeeded(result)
 	   


##  The main function of the script:
#   Creates a publisher on topic /cmd_vel to publish the velocity of the robot
#   Creates a subscriber of topic /odom to receive the position of the robot
#   Creates a subscriber of topic /slider to receive the value of the sliders
#   Creates an action in order to manage the FSM behaviour
##
def main():
    global pub_, server
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_slider=rospy.Subscriber('/slider',Twist, clbk_slider)
    server = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.PositionAction, execute_cb = go_to_point, auto_start=False)
    server.start()
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
	      rate.sleep()

if __name__ == '__main__':
    main()
