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

class TopicPublisherNode : public rclcpp::Node
{
  topic_msgs::msg::Byte_Msg msg_buff;
  rclcpp::Publisher<topic_msgs::msg::Byte_Msg>::SharedPtr
      msg_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  void step()
  {
    long long start_time = getCurrentTimeNanoseconds();
    // do some random calculation
    for (int i = 0; i < 1000; i++)
    {
      msg_buff.data[i] = rand() % 255;
    }
    msg_pub_->publish(msg_buff);
    long long end_time = getCurrentTimeNanoseconds();
    long long nano_cost = end_time - start_time;
    if (nano_cost > 10'000'000ll)
    {
      // greater than 10 ms
      RCLCPP_ERROR(this->get_logger(), "publish took more than 10ms! actual time: %lld ns", nano_cost);
    }

    // RCLCPP_INFO(this->get_logger(), "cost: %lld ns", nano_cost);
  }
  TopicPublisherNode(const std::string node_name,
                     const rclcpp::NodeOptions &options) : Node(node_name, options)
  {
    RCLCPP_INFO(this->get_logger(), "Publisher node creating...");
    msg_pub_ = this->create_publisher<topic_msgs::msg::Byte_Msg>(
        "ecu_test/byte_data", 1);
    timer_ = create_wall_timer(std::chrono::microseconds(10000),
                               std::bind(&TopicPublisherNode::step, this));
    RCLCPP_INFO(this->get_logger(), "Publisher node created.");
  }
};

int main(int argc, char **argv)
{
  static constexpr auto NODE_NAME = "ecu_error_test_publisher_node";
  rclcpp::init(argc, argv);
  auto executor = rclcpp::executors::MultiThreadedExecutor{};
  auto node =
      std::make_shared<TopicPublisherNode>(NODE_NAME, rclcpp::NodeOptions());

  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
