cmake_minimum_required(VERSION 3.5)
project(yolov5_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(OpenCV REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(yolov5_node src/yolov5.cpp)

# Link with OpenCV libraries
target_link_libraries(yolov5_node ${OpenCV_LIBS})

ament_target_dependencies(yolov5_node rclcpp std_msgs sensor_msgs vision_msgs)

install(TARGETS yolov5_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch src yoloFiles
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
