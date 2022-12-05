
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

#include <gtest/gtest.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

class TestNode : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestNode, test_for_publishers) {
  node_ = std::make_shared<rclcpp::Node>("test_publisher");
  auto test_publisher = node_->create_publisher<std_msgs::msg::String>
                    ("chatter", 10.0);

  auto num_of_publishers = node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(num_of_publishers));
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}