#include "rclcpp/rclcpp.hpp"
#include "rcl/time.h"
#include "communication_trace_msgs/msg/communication_trace.hpp"
#include "std_msgs/msg/header.hpp"

class CommTracer : public rclcpp::Node {
 public:
  CommTracer(std::string node_name, std::string topic_name);
  void publish(const std_msgs::msg::Header &header);

 private:
   rclcpp::Publisher<communication_trace_msgs::msg::CommunicationTrace>::SharedPtr pub_;
};
