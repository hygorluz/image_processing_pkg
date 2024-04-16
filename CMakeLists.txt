cmake_minimum_required(VERSION 3.8)
project(image_processing_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCroppedAxis.msg"
  "srv/ImageCrop.srv"
  DEPENDENCIES sensor_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(image_crop_client_node src/image_crop_client_node.cpp)
ament_target_dependencies(image_crop_client_node rclcpp OpenCV cv_bridge image_transport)

add_executable(image_crop_server_node src/image_crop_server_node.cpp)
ament_target_dependencies(image_crop_server_node rclcpp OpenCV cv_bridge image_transport)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(image_crop_client_node "${cpp_typesupport_target}")
target_link_libraries(image_crop_server_node "${cpp_typesupport_target}")

target_include_directories(image_crop_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(image_crop_client_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  image_crop_client_node
  image_crop_server_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
