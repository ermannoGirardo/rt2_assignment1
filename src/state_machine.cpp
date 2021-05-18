#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/PositionAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>


bool start = false;

/**
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
*	main funciton of the state_machine node:
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
   ros::Publisher talk_vel= n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   actionlib::SimpleActionClient <rt2_assignment1::PositionAction> pos("/go_to_point", true);
   rt2_assignment1::PositionGoal goal_pos;
   geometry_msgs::Twist vel;
   bool target_reached=false;
   
   rt2_assignment1::RandomPosition rndpos;
   rndpos.request.x_max = 5.0;
   rndpos.request.x_min = -5.0;
   rndpos.request.y_max = 5.0;
   rndpos.request.y_min = -5.0;


   
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

              break;

           }
            target_reached=true;
        }
	
    }

    else
    {
      //Set the velocity to zero 
            vel.linear.x=0;
            vel.angular.z=0;
            talk_vel.publish(vel);

    }

    //Notify the goal has been reached only in case robot isn't stopped from user
    if(target_reached)
    {
         //Notofy we reached the goal
         std::cout << "\nPosition reached" << std::endl;
         target_reached=false;
    }
           
   	}
   return 0;
}
