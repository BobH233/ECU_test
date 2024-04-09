#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "topic_msgs/msg/byte__msg.hpp"

long long getCurrentTimeNanoseconds()
{
  // Define a clock type with nanosecond precision
  using Clock = std::chrono::high_resolution_clock;

  // Get the current time point
  auto now = Clock::now();

  // Convert the time point to nanoseconds since the epoch
  auto nanoseconds_since_epoch = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch();

  // Convert the duration to a long long value representing nanoseconds
  return static_cast<long long>(nanoseconds_since_epoch.count());
}

class TopicSubscriberNode : public rclcpp::Node
{
  topic_msgs::msg::Byte_Msg msg_buff;
  rclcpp::Subscription<topic_msgs::msg::Byte_Msg>::SharedPtr
      msg_sub_;

public:
  void callback(const topic_msgs::msg::Byte_Msg::SharedPtr msg)
  {
    msg_buff = *msg;
    static long long last_recv_nano = 0;
    if (last_recv_nano == 0)
    {
      last_recv_nano = getCurrentTimeNanoseconds();
      return;
    }
    long long delta_time = getCurrentTimeNanoseconds() - last_recv_nano;
    last_recv_nano = getCurrentTimeNanoseconds();
    if (delta_time > 20'000'000ll)
    {
      RCLCPP_ERROR(this->get_logger(), "subscriber recv more than 20ms! actual time: %lld ns", delta_time);
    }
  }
  TopicSubscriberNode(const std::string node_name,
                      const rclcpp::NodeOptions &options) : Node(node_name, options)
  {
    RCLCPP_INFO(this->get_logger(), "Subscriber node creating...");
    msg_sub_ = this->create_subscription<topic_msgs::msg::Byte_Msg>(
        "ecu_test/byte_data", 1,
        std::bind(&TopicSubscriberNode::callback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscriber node created.");
  }
};

int main(int argc, char **argv)
{
  static constexpr auto NODE_NAME = "ecu_error_test_subscriber_node";
  rclcpp::init(argc, argv);
  auto executor = rclcpp::executors::MultiThreadedExecutor{};
  auto node =
      std::make_shared<TopicSubscriberNode>(NODE_NAME, rclcpp::NodeOptions());

  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
