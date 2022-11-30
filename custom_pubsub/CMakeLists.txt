cmake_minimum_required(VERSION 3.8)
project(custom_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

# publisher executable
add_executable(talker src/publisher_node.cpp)
ament_target_dependencies(talker 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros

)


# subscriber executable
add_executable(listener src/subscriber_node.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# custom message lines
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModifyString.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
"rosidl_typesupport_cpp" 
)

target_link_libraries(talker "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

# targets for ros2
install(TARGETS
   talker
   listener
   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
