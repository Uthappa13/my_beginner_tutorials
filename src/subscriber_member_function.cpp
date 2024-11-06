/**
 * @file subscriber_member_function.cpp
 * @author Uthappa Madettira (120305085)
 * @brief
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Minimal Subscriber Node that listens to /topic and echoes it to
 * RCLCPP_INFO logging level
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object and instantiate the
   * subscriber object for the /topic with the topic_callback
   *
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        std::bind(&MinimalSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

 private:
  /**
   * @brief /topic subscriber callback that echoes the message to RCLCPP_INFO
   * logging level
   *
   * @param msg
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
