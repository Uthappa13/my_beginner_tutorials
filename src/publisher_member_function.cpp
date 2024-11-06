/**
 * @file publisher_member_function.cpp
 * @author Uthappa Madettira (120305085)
 * @brief
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <beginner_tutorials/srv/change_str.hpp>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

/**
 * @brief PublisherandService node to act as a simple publisher and a service to
 * change the published string. The publish rate is determined by a parameter
 *
 */

class PublisherandServiceNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Publisherand Service Node object and instantiate the
   * publisher, subscriber and the timer object that calls the publisher
   *
   */
  PublisherandServiceNode() : Node("minimal_publisher"), count_(0) {
    this->declare_parameter("publish_frequency", 500);
    this->message.data = "Unmodified Message, Use service to change";
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeStr>(
        "change_string",
        std::bind(&PublisherandServiceNode::change_str, this,
                  std::placeholders::_1, std::placeholders::_2));
    if (this->get_parameter("publish_frequency").as_int() == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Publisher frequency has not changed, is still : "
              << this->get_parameter("publish_frequency").as_int());
    }
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publish_frequency").as_int()),
        std::bind(&PublisherandServiceNode::timer_callback, this));
  }

 private:
  /**
   * @brief Timer callback that publishes the message. Publish frequency is
   * determined by the parameter.
   *
   */
  void timer_callback() {
    auto message = this->message;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:" << message.data);
    publisher_->publish(message);
  }
  /**
   * @brief Change string callback from the service, changes the string being
   * published
   *
   * @param resp Response echoes the input string
   */
  void change_str(
      const std::shared_ptr<beginner_tutorials::srv::ChangeStr::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeStr::Response> resp) {
    this->message.data = request->new_string;
    resp->string_change_status = request->new_string;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Received Service Request: " << request->new_string);
  }
  std_msgs::msg::String message;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeStr>::SharedPtr service_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherandServiceNode>());
  rclcpp::shutdown();
  return 0;
}
