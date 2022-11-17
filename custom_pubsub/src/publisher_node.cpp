// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_pubsub/srv/modify_string.hpp"

using namespace std::chrono_literals;



/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
  : Node("publisher_node"), count_(0) {
    // set the logger level to DEBUG from Info
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                    "Getting frequency parameter value");

    // Parameter for initializing publisher frequency  with custom frequency
    auto custom_pubfreq_info = rcl_interfaces::msg::ParameterDescriptor();
    custom_pubfreq_info.description = "Custom frequency"+
                                     "value for the publisher";
    this->declare_parameter("custom_pubfreq", 1.0, custom_pubfreq_info);
    auto custom_pubfreq = this->get_parameter("custom_pubfreq")
                  .get_parameter_value().get<std::float_t>();

    // Checking frquency val and using logging to display errors
    if (custom_pubfreq < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                  "Publisher frequency cannot be negative!");
      } else if (custom_pubfreq == 0) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                  "Frequency set to zero!!");
      }

    publisher_ = this->create_publisher<std_msgs::msg::String>
                 ("topic", 10);
    // calculate publisher freq
    auto time = std::chrono::milliseconds(
            static_cast<int>(1000/custom_pubfreq));

    timer_ = this->create_wall_timer(
      time, std::bind(&MinimalPublisher::timer_callback, this));

    auto serviceCallbackPtr = std::bind(&MinimalPublisher::modify_message,
                    this, std::placeholders::_1, std::placeholders::_2);

    service_ = create_service<custom_pubsub::srv::ModifyString>(
                    "modify_message", serviceCallbackPtr);
    }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hi this is Sairam, talking for "+
    std::to_string(count_++)+ " time.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
  }

  void modify_message(
    const std::shared_ptr<custom_pubsub::srv::ModifyString::Request> request,
    std::shared_ptr<custom_pubsub::srv::ModifyString::Response> response) {
    response->response_message = request->request_message+
             " hi, this is not request message!!!";
    RCLCPP_INFO_STREAM(this->get_logger(),
                  "Request message: "<< request->request_message);
    RCLCPP_INFO_STREAM(this->get_logger(),
                  "Response message: "<< response->response_message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<custom_pubsub::srv::ModifyString>::SharedPtr service_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
