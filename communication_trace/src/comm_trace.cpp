#include "communication_trace/comm_trace.hpp"
#include <algorithm>

namespace communication_trace {
std::string normalize(std::string name) {
  std::replace(name.begin(), name.end(), '/', '_');
  return name;
}

CommTrace::CommTrace(std::string node_name,
                                       std::string topic_name)
    : Node(normalize(node_name) + normalize(topic_name),
           "communication_trace") {
  pub_ = create_publisher<communication_trace_msgs::msg::CommunicationTrace>(
      get_name(), 1);
}

void CommTrace::publish(const std_msgs::msg::Header &header) {
  // TODO: print error when ComTracer isn't initialized
  auto msg_ =
      std::make_unique<communication_trace_msgs::msg::CommunicationTrace>();
  msg_->header = header;
  msg_->stamp = now();
  pub_->publish(std::move(msg_));
}
} // namespace communication_trace
