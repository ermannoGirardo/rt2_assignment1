#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cinttypes>
#include <cstdlib>

#include"rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

namespace rt2_assignment1
{
    
	class FsmServer : public rclcpp::Node
	{
	   public:
		
		/**
		*	Constructor of the class FsmServer which must be considered as a component
		*	Here you can find the initialisation of
		*		-client1: in order to ask a new random goal at custom service RandomPosition
		*		-cllient2: in order to set the random goal generated as a goal position and send it to /go_to_point
		*		-service: in order to read the command sent by the user
		*
		*/
		
		FsmServer(const rclcpp::NodeOptions & options)
		: Node("fsmserver",options)
		{
		     //Initialize client1 client2 and service
		     client1=this->create_client<rt2_assignment1::srv::RandomPosition>("/position_server");
		     while(!client1->wait_for_service(std::chrono::seconds(3))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "random position client interrupted");
						return ;
					}
					RCLCPP_INFO(this->get_logger(), "waiting for random service...");
				}
		    
		     client2=this->create_client<rt2_assignment1::srv::Position>("/go_to_point");
		     while(!client2->wait_for_service(std::chrono::seconds(3))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "position client interrupted");
						return ;
					}
					RCLCPP_INFO(this->get_logger(), "waiting for position service...");
				}
		     service = this->create_service<rt2_assignment1::srv::Command>("/user_interface",std::bind(&FsmServer::user_interface, this, _1, _2, _3));
		}
		
		
		/**
		*	Function used to make a request to the RandomPosition service in order to generate a random number.
		*	It is also used to get the result of the RandomPosition service and send it to /go_to_point service
		*	Check the value of the variable start, if it is true tbe function makes a request to the RandomPosition server
		*	in order to generate a new random goal
		*	Once the new random goal is generated the funcion send it to go_to_point and waits the result
		*/
	        void call_clients(){
		     auto request = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
		     goal_position = std::make_shared<rt2_assignment1::srv::Position::Request>();
		     request->x_max=5.0;
		     request->x_min=-5.0;
		     request->y_max=5.0;
		     request->y_min=-5.0;
		     if(this->start){		     
		     	using ServiceResponseFuture =
    				rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture;
    		        auto response_received_callback = [this](ServiceResponseFuture future){
 		     		goal_position->x=future.get()->x;
 		     		goal_position->y=future.get()->y;
 		     		goal_position->theta=future.get()->theta;	
 		     		std::cout << "\nGoing to the position: x = " << goal_position->x << " y = " << goal_position->y << " theta = " << goal_position->theta << std::endl;
 		     		using ServicePositionFuture = rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture;
				auto response = [this] (ServicePositionFuture future_position){
					if(future_position.get()->ok){
				   				std::cout << "Goal Reached!" << std::endl;
				   				call_clients();
				   			}
				   	};
 		     		auto result_future = client2->async_send_request(goal_position,response);
			  };
		     	auto result_future2 = client1->async_send_request(request,response_received_callback);
			}
		}		
		
            private:
               bool start = false;
               
               
     		/**
     		*	Function call_back of the user_interface server
     		*	Based on the selected request of the user this function properly set the variable start
     		*
     		*/
		bool user_interface(
			 const std::shared_ptr<rmw_request_id_t> request_header,
  			 const std::shared_ptr<rt2_assignment1::srv::Command::Request> request,
  			 const std::shared_ptr<rt2_assignment1::srv::Command::Response> response)
  		{
 			(void)request_header;
   			if (request->command == "start"){
    				this->start = true;
    				this->call_clients();
    			}
    			else {
    				this->start = false;
    			}
    			return true;
		}
	        rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr client1;
                rclcpp::Client<rt2_assignment1::srv::Position>::SharedPtr client2;
                rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr service;
                std::shared_ptr<rt2_assignment1::srv::Position::Request> goal_position;
          };
}



RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::FsmServer)

