/**
 * @file publisher_member_function.cpp
 * @author Uthappa Madettira
 * @brief
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <beginner_tutorials/srv/change_str.hpp>
#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger("");  // Create an initial Logger

class MyTestsFixture {
 public:
  MyTestsFixture() {
    /**
     * 1.) Create the node that performs the test. (aka Integration test node):
     */
    testerNode = rclcpp::Node::make_shared("IntegrationTestNode1");
    Logger =
        testerNode
            ->get_logger();  // Make sure message will appear in rqt_console

    /**
     * 2.) Declare a parameter for the duration of the test:
     */
    testerNode->declare_parameter<double>("test_duration");

    /**
     * 3.) Get the test duration value:
     */
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration = " << TEST_DURATION);
  }

  ~MyTestsFixture() {}

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

/* In this test case, the node under test (aka Auxiliary test node)
   is a service server, which got launched by the launcher.

   We will create a service client as part of the node performing the
   test (aka Integration test node). The test simply checks if
   the service is available within the duration of the test. */

TEST_CASE_METHOD(MyTestsFixture, "test service server", "[service]") {
  /**
   * 4.) Now, create a client for the specific service we're looking for:
   */
  auto client = testerNode->create_client<beginner_tutorials::srv::ChangeStr>(
      "change_string");
  RCLCPP_INFO_STREAM(Logger, "'change_string' client created");

  /**
   * 5.) Finally do the actual test:
   */
  rclcpp::Time start_time =
      rclcpp::Clock().now();  // Reads /clock, if "use_sim_time" is true
  bool service_found = false;
  rclcpp::Duration duration = 0s;
  RCLCPP_INFO_STREAM(Logger, "Performing Test...");
  auto timeout = std::chrono::milliseconds(static_cast<int>(TEST_DURATION * 1000));

  if (client->wait_for_service(timeout)) {  // Blocking
    duration = (rclcpp::Clock().now() - start_time);
    service_found = true;
  }

  RCLCPP_INFO_STREAM(Logger,
                     "duration = " << duration.seconds()
                                   << " service_found = " << service_found);
  CHECK(service_found);  // Test assertions - check that the service was found
}

////////////////////////////////////////////////
// Test Case 2
////////////////////////////////////////////////

/* In this test case, the node under test (aka Auxiliary test node)
   is a topic talker, which got launched by the launcher.

   We will create a topic listener as part of the node performing the
   test (aka Integration test node). The test simply checks if
   the topic is received within the duration of the test. */

TEST_CASE_METHOD(MyTestsFixture, "test topic talker", "[topic]") {
  /**
   * 4.) Now, subscribe to a specific topic we're looking for:
   */
  bool got_topic = false;

  // Define a callback that captures the additional parameter
  struct ListenerCallback {
    explicit ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic) {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM(Logger, "I heard: " << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  auto subscriber = testerNode->create_subscription<String>(
      "topic", 10, ListenerCallback(got_topic));
  RCLCPP_INFO_STREAM(Logger, "Subscribed to 'topic'.");

  /**
   * 5.) Finally do the actual test:
   */
  rclcpp::Rate rate(10.0);  // 10Hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);
  RCLCPP_INFO_STREAM(
      Logger, "duration = " << duration.seconds()
                            << " timeout = " << timeout.seconds());

  while (!got_topic && (duration < timeout)) {
    rclcpp::spin_some(testerNode);
    rate.sleep();
    duration = (rclcpp::Clock().now() - start_time);
  }

  RCLCPP_INFO_STREAM(
      Logger, "duration = " << duration.seconds()
                            << " got_topic = " << got_topic);
  CHECK(got_topic);  // Test assertions - check that the topic was received
}


