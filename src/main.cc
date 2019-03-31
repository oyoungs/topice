

#include <oyoung/topice.hpp>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <iostream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Topice");
  ros::NodeHandle node;

  auto  topice = oyoung::make_topice(node);

  topice.service<std_msgs::Bool, std_msgs::String>("/setBool", 
        [=](const std_msgs::Bool& request, std_msgs::String& response) {
          response.data = "success";
          std::cout << "service called" << std::endl;
          return true;
        }
  );

  std::thread background([&] {

    while(ros::ok()) {
      std_msgs::Bool request;
      request.data = true;
      std_msgs::String response;
      if(oyoung::Status::Success  == topice.call( "/setBool",request, response, std::chrono::seconds(10))) {
        std::cout << "success" << std::endl;
      } else {
        std::cerr << "timeout" << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });
  ros::spin();
  
  return 0;
}
