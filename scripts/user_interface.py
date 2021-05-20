"""
/user_interface.py

/brief implements an user interface in order to start or stop the robot. In particular it is a client that reads the user command and via custom service message Command.srv send it to the state_machine node. When the user press 1 the node sends a "start" messagge to the user_interface server. Instead if the user press 0 the node the function of go_to_point is stopped and in state_machine node the goal is cancelled, setting also the velocity to zero.

/author Ermanno Girardo 

/date May 2021
"""

import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Robot has been stopped")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
