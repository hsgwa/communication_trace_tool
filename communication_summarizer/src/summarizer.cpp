#include <chrono>
#include <memory>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "communication_trace_msgs/msg/communication_trace.hpp"

using namespace std::chrono_literals;

// topic の一覧を取得
// subを作成
// 各subで集計
// 終了直前で保存

class SummarizerNode : public rclcpp::Node {
 public:
  SummarizerNode() : Node("summarizer") {
    auto topic_names_and_types = get_topic_names_and_types();
    for (std::pair<std::string, std::vector<std::string>> topic_name_and_type :
         topic_names_and_types) {
      auto topic_name = topic_name_and_type.first;
      auto types = topic_name_and_type.second;

      std::cout << topic_name << std::endl;
      for (auto &type : types) {
        std::cout << type << std::endl;
      }
      std::cout << "itemized all topics" << std::endl;
    }
  }
  private:
  std::vector<rclcpp::Subscription<communication_trace_msgs::msg::CommunicationTrace>> subs_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SummarizerNode>());
  rclcpp::shutdown();
  return 0;
}
