
/*****************************************************************//**
 * \file   position_service.cpp
 * \brief Simple server that has the scope of generate a random target.
 	 This node infact generate three random components 
 	 (x and y components for the position, theta 
 	 component for the orientation) in a specified interval 
 	 [min,max] via custom service message RandomPosition.srv.
 *
 * \author Girardo Ermanno
 * \date   May 2021
***********************************************************************/



#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"


double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
