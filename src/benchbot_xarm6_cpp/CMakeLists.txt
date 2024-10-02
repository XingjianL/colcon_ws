cmake_minimum_required(VERSION 3.8)
project(benchbot_xarm6_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
endif()

#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
#set(CMAKE_PREFIX_PATH "/home/lxianglabxing/open3d_install/lib/cmake/Open3D" ${CMAKE_PREFIX_PATH})
set(Open3D_DIR "/home/lxianglabxing/open3d_install/lib/cmake/Open3D")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)

find_package(Open3D REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
#link_directories(${Open3D_LIBRARY_DIRS})

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  include
)

set(SOURCES
    src/xarm6_moveit.cpp
    src/csv_parser.cpp
    src/image_subscribe.cpp
    src/benchbot_xarm6_cpp.cpp
    src/unique_point_cloud.cpp
    src/environment_info.cpp
    src/planar_robot.cpp
    src/nbv.cpp
    # add other source files here
)
add_executable(benchbot_xarm6_cpp ${SOURCES})

target_include_directories(benchbot_xarm6_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(benchbot_xarm6_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  benchbot_xarm6_cpp
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  #message_filters
  #Open3D
)

install(TARGETS benchbot_xarm6_cpp
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(benchbot_xarm6_cpp Open3D::Open3D)


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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
