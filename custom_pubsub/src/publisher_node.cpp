// Copyright Venkata Sai Ram Polina
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

/**
 * @file publisher_node.cpp
 * @author Sairam Polina (sairamp@umd.edu)
 * @brief Illustration of how to write basic ROS2 publisher subscriber and srvics, and logging
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_pubsub/srv/modify_string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

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
    custom_pubfreq_info.description =
                   "Custom frequency value for the publisher";
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
         RCLCPP_FATAL_STREAM(this->get_logger(), "Not publishing Data!!");
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
 /**
  * @brief callback funtion which calls the publisher according to set frequency
  * 
  */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hi this is Sairam, talking for "+
    std::to_string(count_++)+ " time.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
  }

  /**
   * @brief service to modify  a string message 
   * 
   * @param request string input message ptr
   * @param response string output message ptr
   */
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
  /**
   * @brief private data members
   * 
   */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<custom_pubsub::srv::ModifyString>::SharedPtr service_;
  size_t count_;
};

/**
 * @brief This class publishes a static frame to /tf_static topic
 * 
 */
class StaticFramePublisher : public rclcpp::Node {
 public:
  explicit StaticFramePublisher(char * transformation[])
  : Node("publisher_node") {
    tf_static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms(transformation);
  }

 private:
  void make_transforms(char * transformation[]) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(
      atof(transformation[5]),
      atof(transformation[6]),
      atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};




int main(int argc, char * argv[]) {
  // To run publisher only
  if (argc ==1) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  } else {
    // to run  static broadcaster
    auto logger = rclcpp::get_logger("logger");

    // Obtain parameters from command line arguments
    if (argc != 8) {
      RCLCPP_INFO(
        logger, "Invalid number of parameters\nusage: "
        "$ ros2 run custom_pubsub talker "
        "child_frame_name x y z roll pitch yaw");
      return 1;
    }

    // As the parent frame of the transform is `world`, it is
    // necessary to check that the frame name passed is different
    if (strcmp(argv[1], "world") == 0) {
      RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
      return 2;
    }


    rclcpp::init(argc, argv);
    RCLCPP_INFO(logger,
                    "Starting Tf2 Static Brodcaster");
    rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
    rclcpp::shutdown();
    return 3;
  }
}
