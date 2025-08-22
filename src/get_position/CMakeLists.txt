cmake_minimum_required(VERSION 3.8)
project(get_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lcm REQUIRED)
find_package(self_interface REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(./include/get_position)

add_executable(pub_position src/pub_position.cpp)
add_executable(pub_heart src/pub_heart.cpp)
ament_target_dependencies(pub_position 
  rclcpp 
  self_interface 
  std_msgs)
ament_target_dependencies(pub_heart 
rclcpp 
std_msgs)
target_link_libraries(pub_position lcm) 

install(TARGETS
  pub_position
  pub_heart
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
