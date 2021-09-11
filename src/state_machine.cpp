
/**
* \file   state_machine.cpp
* \brief  Develop a state machine in order to manage the simulation
*
* \author Carmine Recchiuto, Ermanno Girardo
*
* \version 1,2
*
* \date   September 2021
*
* \details
*
* Publishes to <BR>
*	/goalResult
*
* Client: <BR>
*	/user_interface
*
* Action client: <BR>
*	/position_server
*
* Description:
*         state_machine.cpp is a node wich communicates with all other nodes, in particular:
*	  1) a client is generated in order to set the fields of the random target and send the request to position_service
*	  2) The node implements a function callback of the user_interface , in order to check the request done by the user, 
*	     setting properly the value of the variable start.
*	  3) Implements an action client in order to cancel the goal if the user decides to stop the robot.
*	  4) Publish on topic /cmd_vel the velocity equal to zero if the user decides to stop the robot.
*/


#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/PositionAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int32.h"


bool start = false;

/**
*  bool user_interface (req,res)
*	
*	\brief Function used to change the status of the robot ( start or stop)
*
*	\param req: is the request done by the client, in particular start or stop the robot
*
*	\param res: is the response from the server.This response is true if the server finishes to set
*		    the variable start
*
*	\return true if the server finishes to set the start value
*
*	Description:
*	function that user interface server use in order to modify the value of the variable start
*	if the client makes the request to 'start' this function set to true the start variable
*	Otherwise is the to false
*/

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
*	\brief main funciton of the state_machine node
*
*	\param argc:number of arguments passed as parameters
*
*	\param argv:vector of string containing each argument
*
*	\return 0 when the program ends
*
*	Description:
*	-create the service at /user_interface and the function callback user_interface
*	-create a client for the random position target in order to generate a new random target
*	-create a publisher on topic /cmd_vel in order to send the correct robot velocity			
*	-create an action client in order to send the position of the goal and if necessary to cancel all goals
*	-the action client is able to check if the goal is reached
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::Publisher goals_pub = n.advertise<std_msgs::Int32>("/goalResult", 1000);
   actionlib::SimpleActionClient <rt2_assignment1::PositionAction> pos("/go_to_point", true);
   rt2_assignment1::PositionGoal goal_pos;
   geometry_msgs::Twist vel;
   bool target_reached=false;
   
   rt2_assignment1::RandomPosition rndpos;
   rndpos.request.x_max = 5.0;
   rndpos.request.x_min = -5.0;
   rndpos.request.y_max = 5.0;
   rndpos.request.y_min = -5.0;

   std_msgs::Int32 condition;

   
    while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   	client_rp.call(rndpos);
        goal_pos.x=rndpos.response.x;
        goal_pos.y=rndpos.response.y;
        goal_pos.theta=rndpos.response.theta;

        std::cout << "\nGoing to the position: x= " << goal_pos.x << " y= " <<goal_pos.y << " theta = " <<goal_pos.theta << std::endl;
        pos.sendGoal(goal_pos);


        while (pos.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
           ros::spinOnce();
           if(start==false)
           {
              //Cancel all the goals
              pos.cancelAllGoals();
              target_reached=false;
	      std::cout << "Goal Canceled" << std::endl;
	      condition.data = 0;
	      goals_pub.publish(condition);
              break;

           }
            target_reached=true;
        }
	
    }


    //Notify the goal has been reached only in case robot isn't stopped from user
    if(target_reached)
    {
         //Notofy we reached the goal
         std::cout << "\nPosition reached" << std::endl;
         target_reached=false;
         condition.data = 1;
	 goals_pub.publish(condition);
    }
           
   }
   return 0;
}
