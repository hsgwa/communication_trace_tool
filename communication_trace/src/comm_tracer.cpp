#include "communication_trace/comm_tracer.hpp"
#include <algorithm>

std::string normalize(std::string name) {
  std::replace(name.begin(), name.end(), '/', '_');
  return name;
}

CommTracer::CommTracer(std::string node_name, std::string topic_name)
    :Node(normalize(node_name) + normalize(topic_name), "communication_trace") {
    pub_ = create_publisher<communication_trace_msgs::msg::CommunicationTrace>(get_name(), 1);
}

void CommTracer::publish(const std_msgs::msg::Header &header) {
  auto msg_ =
      std::make_unique<communication_trace_msgs::msg::CommunicationTrace>();
  msg_->header = header;
  msg_->stamp = now();
  pub_->publish(std::move(msg_));
}
