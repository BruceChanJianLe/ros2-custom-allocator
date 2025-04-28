cmake_minimum_required(VERSION 3.8)
project(ros2-custom-allocator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
set(dependencies
  rclcpp
  std_msgs
)

foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()

include(FetchContent)

# pmrspy
FetchContent_Declare(
  pmrspy
  GIT_REPOSITORY https://github.com/brucechanjianle/pmrspy.git
  GIT_TAG master
)
FetchContent_MakeAvailable(pmrspy)

# ros2 example
# https://github.com/ros2/demos/blob/jazzy/demo_nodes_cpp/src/topics/allocator_tutorial_pmr.cpp
add_executable(ros2_example
  ./src/ros2_example.cpp
)
target_include_directories(ros2_example PUBLIC include)
ament_target_dependencies(ros2_example ${dependencies})

# pmr_simple_example
add_executable(pmr_simple_example
./src/pmr_simple_example.cpp
)
target_include_directories(pmr_simple_example PUBLIC include)
ament_target_dependencies(pmr_simple_example ${dependencies})
target_link_libraries(pmr_simple_example pmrspy::pmrspy)

install(TARGETS
  ros2_example
  pmr_simple_example
  DESTINATION lib/${PROJECT_NAME}
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
