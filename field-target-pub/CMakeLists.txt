cmake_minimum_required(VERSION 3.8)
project(field_target_pub LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FieldTarget.msg" "msg/TargetSet.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces)

option(BUILD_RUNTIME_NODES "Build fixed target runtime node" ON)
if(BUILD_RUNTIME_NODES)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(field_target_node src/field_target_node.cpp)
rosidl_get_typesupport_target(cpp_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(field_target_node "${cpp_typesupport}" nlohmann_json::nlohmann_json)

ament_target_dependencies(field_target_node
  rcl_interfaces
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
)

target_include_directories(field_target_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  field_target_node
  DESTINATION lib/${PROJECT_NAME})

endif()

  install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING AND BUILD_RUNTIME_NODES AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/field_target_test.py")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(field_target_test test/field_target_test.py
    ENV FIELD_TARGET_EXECUTABLE=$<TARGET_FILE:field_target_node>
    TIMEOUT 60)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()