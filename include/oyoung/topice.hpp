#ifndef OYOUNG_TOPICE_H
#define OYOUNG_TOPICE_H

#include <ros/node_handle.h>
#include <string>
#include <chrono>
#include <future>
// #include <iostream>

namespace oyoung {
  
  struct Server {
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    Server() {}
    Server(const ros::Publisher& pub, const ros::Subscriber& sub): publisher(pub), subscriber(sub) {}

    template<typename MSG>
    void advertise(const std::string& topic, std::size_t queue_size,  ros::NodeHandle& node) {
      publisher = node.advertise<MSG>(topic, queue_size);
    }

    template<typename MSG>
    void publish(const MSG& msg) { publisher.publish(msg); }

    template<typename MSG, typename Func>
    void subscribe(const std::string& topic, std::size_t queue_size, Func&& func, ros::NodeHandle& node) {
      subscriber = node.subscribe<MSG>(topic, queue_size, std::move(func));
    }
    
  };

  enum class Status {
    Success,
    Timeout
  };

  template<typename Node>  
  struct Topice {
    
    Topice(ros::NodeHandle& node): _node(node) {}


    template<typename MReq, typename MRes>
    std::shared_ptr<Server> service(const std::string& service, bool (*func)(const MReq&, MRes&)) {
      auto request = service + "/request";
      auto response = service + "/response";

      auto server = std::make_shared<Server>();

      server->advertise<MRes>(response, 10, _node);
      server->subscribe<MReq>(request, 10, [=](const typename MReq::ConstPtr& msg) {
          MRes responseMsg;
          if(func(*msg, responseMsg)) {
            server->publish(responseMsg);
          }
      }, _node);


      return server;
    } 



    template<typename MReq, typename MRes, typename Rep, typename Period>
    Status call(const std::string& service, const MReq& request, MRes& response, const std::chrono::duration<Rep, Period>& duration) {

        auto requestTopic = service + "/request";
        auto responseTopic = service + "/response";
        auto promise = std::make_shared<std::promise<MRes>>();
        auto future = promise->get_future();

        auto subscriber = _node.subscribe<MRes>(responseTopic, 10, [=](const typename MRes::ConstPtr& res) {
            // std::cout << "subscribed topic: " << responseTopic << std::endl;
            promise->set_value(*res);
        });

        auto publisher = _node.advertise<MReq>(requestTopic, 10);
        publisher.publish(request);

        // std::cout << "publish" << std::endl; 

        auto status =  future.wait_for(duration);
        if(std::future_status::ready == status) {
          response = future.get();
        } else if(std::future_status::timeout == status) {
          return Status::Timeout;
        }

        return Status::Success;

    }
    

  private:
    ros::NodeHandle& _node;
  };

  template<typename NodeHandle>
  Topice<NodeHandle> make_topice(NodeHandle& node) {
    return Topice<NodeHandle>(node);
  }
}

#endif /* OYOUNF_TOPICE_H */