cmake_minimum_required(VERSION 3.8)
project(pick_place_robust)

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
find_package(moveit_msgs REQUIRED)

include_directories(include)
FILE(GLOB_RECURSE INCLUDE_FILES "include/pick_place_robust/*.hpp")

add_executable(pick_place_robust src/pick_place_robust.cpp src/one_grasp_utils.cpp)
target_sources(pick_place_robust PRIVATE ${INCLUDE_FILES})
target_include_directories(pick_place_robust PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_place_robust PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pick_place_robust
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "gazebo_msgs"
  "tf2"
  "moveit_msgs"
)

install(TARGETS pick_place_robust
  DESTINATION lib/${PROJECT_NAME})

# Install the include directory
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

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
