cmake_minimum_required(VERSION 3.8)
project(controller)

# # 设置C++标准
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
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
find_package(self_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(toml REQUIRED)
find_package(lcm REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(yaml-cpp REQUIRED)

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

include_directories(./include/controller)

# 声明节点
add_executable(fsm src/fsm.cpp)
# add_executable(subc_pos src/subc_pos.cpp)

ament_target_dependencies(fsm 
  rclcpp
  std_msgs
  toml
  self_interface 
  geometry_msgs
  cv_bridge
  lcm#这个的依赖到底是target_link还是ament_target_dependencies我还不确定
  tf2
  tf2_geometry_msgs
  tf2_msgs
)
# ament_target_dependencies(subc_pos 
#   rclcpp 
#   self_interface 
#   std_msgs)

# target_link_libraries(subc_pos lcm) 
target_link_libraries(fsm 
  lcm
) 
#target_link_libraries(fsm toml_library)这一行是错误的
# 安装目标
install(TARGETS 
  # subc_pos
  fsm
  DESTINATION lib/${PROJECT_NAME}
)

# 安装包含文件  这个方法不行
# install(DIRECTORY include/
#   DESTINATION include/
# )

ament_package()
