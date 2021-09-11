
/**
 * \file   position_service.cpp
 *
 * \brief  generate three random double for the x y theta coordinates
 *
 * \author Carmine Recchiuto, Ermanno Girardo
 *
 * \version 1.2
 *
 * \date   September 2021
 *
 * description:
 *  	 Simple server that has the scope of generate a random target.
 *	 This node infact generate three random components 
 *	 (x and y components for the position, theta 
 *	 component for the orientation) in a specified interval 
 *	 [min,max] via custom service message RandomPosition.srv.
 */



#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"


/**
*   randMtoN (double M, double N)
*
*   \brief generate a random double into the interval [M,N]
* 
*   \param M is the minimum accettable value
*
*   \param N is the maximum accettable value
*
*   \return random double into the interval [M,N]
*/
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
* bool myrandom(res,req)
*
*   \brief calling the randMtoN func three times
*          reply the random x y theta coordinates
*
*   \param req: request done by the client, that specify the min and max value (M, N)
*
*   \param res: the server reply the three random coordinates
*
*   \return a bool=true if the function has been coorectly stored the coordinates
*/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
* int main(int argc, char argv)
*
*   \brief main function that initialize the server
*
*   \param argc: number of arguments passed as parameters
*
*   \param argv: vector of string containing each argument
*
*   \return 0 when the program ends
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
