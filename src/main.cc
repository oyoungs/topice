

#include <oyoung/topice.hpp>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Topice");
  ros::NodeHandle node;

  auto  topice = oyoung::make_topice(node);

  topice.service<std_msgs::Bool, std_msgs::String>("/setBool", 
        [=](const std_msgs::Bool& request, std_msgs::String& response) {
          response.data = "success";
          return true;
        }
  );


  ros::spin();
  
  return 0;
}
