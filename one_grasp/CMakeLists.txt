cmake_minimum_required(VERSION 3.8)
project(one_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
include_directories(include)
FILE(GLOB_RECURSE INCLUDE_FILES "include/one_grasp/*.hpp")

add_executable(one_grasp src/one_grasp.cpp src/one_grasp_utils.cpp)
target_sources(one_grasp PRIVATE ${INCLUDE_FILES})

target_include_directories(one_grasp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(one_grasp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  one_grasp
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "gazebo_msgs"
  "tf2"
)

# Install the executable
install(TARGETS one_grasp
        DESTINATION lib/${PROJECT_NAME}
)

# Install the include directory
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY launch DESTINATION  share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
