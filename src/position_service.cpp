#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/random_position.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

	class PositionService : public rclcpp::Node
	{
		public:
		
			/**
			*	Position service constructor of the class PositionService: must be a componen
			*	Create new custom service of type RandomPosition on /position_server
			*
			*/
		
		
			PositionService(const rclcpp::NodeOptions & options) : Node("random_position_server", options){
				// creating a new service for RandomPosition
				service = this->create_service<rt2_assignment1::srv::RandomPosition>("/position_server",std::bind(&PositionService::myrandom, this, _1, _2, _3));
			}
			
		private:
			/**
			*	Function used to generate a random number between the interval [M,N]
			*	Parameters:
			*		M: the minimum value of the intervall
			*		N: the maximum value of the intervall
			*	Return:
			*		Random number between [M,N]
			*			é
			*/
			
			double randMToN(double M, double N)
			{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }
			
			/**
			*	Function used by the server in order to set the coordinates of the random goal
			*	Param:
			*		req: it is the request done by the client, which is composed of the minimum and maximum value of the x,y (position) and theta (orientation) components
			*		res: it is the response done by the server, which set the three components of the random goal
			*/
			
			bool myrandom (const std::shared_ptr<rmw_request_id_t> request_header,
						   const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> req,
						   const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> res){
				(void)request_header;
				res->x = randMToN(req->x_min, req->x_max);
				res->y = randMToN(req->y_min, req->y_max);
				res->theta = randMToN(-3.14, 3.14);
				return true;
			}
			
			rclcpp::Service<rt2_assignment1::srv::RandomPosition>::SharedPtr service;
	};
}

///Used to specify that PositionService must be considered as a component
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::PositionService)

	
