#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

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
  rclcpp::TimerBase::SharedPtr timer_;

public:
  void step()
  {
    for (int i = 0; i < 25000; i++)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 50));
  }
  TopicPublisherNode(const std::string node_name,
                     const rclcpp::NodeOptions &options) : Node(node_name, options)
  {
    RCLCPP_INFO(this->get_logger(), "Bad node creating...");
    timer_ = create_wall_timer(std::chrono::microseconds(10000),
                               std::bind(&TopicPublisherNode::step, this));
    RCLCPP_INFO(this->get_logger(), "Bad node created.");
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
