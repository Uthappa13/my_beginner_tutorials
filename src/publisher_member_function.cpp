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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <beginner_tutorials/srv/change_str.hpp>
#include <chrono>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

class TalkerTransformPublisherNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new TalkerTransformPublisherNode Node object and
   * instantiate the publisher, subscriber and the timer object that calls the
   * publisher
   *
   */
  TalkerTransformPublisherNode() : Node("transform_publisher") {
    this->declare_parameter("publish_frequency", 500);
    this->message.data = "Defaut Message";
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeStr>(
        "change_string",
        std::bind(&TalkerTransformPublisherNode::change_str, this,
                  std::placeholders::_1, std::placeholders::_2));
    if (this->get_parameter("publish_frequency").as_int() == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Publisher frequency has not changed, is still : "
              << this->get_parameter("publish_frequency").as_int());
    }
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publish_frequency").as_int()),
        std::bind(&TalkerTransformPublisherNode::timer_callback, this));
  }

 private:
  /**
   * @brief Timer callback that publishes the message. Publish frequency is
   * determined by the parameter.
   *
   */
  void timer_callback() {
    if (this->get_parameter("publish_frequency").as_int() < 100) {
      RCLCPP_ERROR_STREAM_THROTTLE(
          this->get_logger(), *this->get_clock(), 100,
          "Publishing too fast, change publish_frequency parameter");
    } else if (this->get_parameter("publish_frequency").as_int() > 1000) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "Too slow, FATAL, have to change publish_frequency parameter");
    }
    auto message = this->message;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:" << message.data);
    publisher_->publish(message);
    this->publish_talk_transform();
  }
  /**
   * @brief Change string callback from the service, changes the string being
   * published
   *
   * @param request Input string that changes the published string
   * @param resp The same string is echoed back when the string is set
   * successfully
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

  void publish_talk_transform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.1;
    t.transform.translation.z = 0.1;
    tf2::Quaternion q;
    q.setRPY(0.1, 0.1, 0.1);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }
  std_msgs::msg::String message;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeStr>::SharedPtr service_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerTransformPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
