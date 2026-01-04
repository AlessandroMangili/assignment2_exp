cmake_minimum_required(VERSION 3.8)
project(assignment2_exp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(aruco_opencv_msgs REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(marker_service_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(get_plan src/getplan.cpp)
add_executable(get_plan_and_execute src/getplan_and_execute.cpp)
add_executable(rotation_action_node src/rotation_action_node.cpp)
add_executable(move_to_waypoint_action src/move_to_waypoint_action.cpp)
add_executable(check_end_action_node src/check_end_action_node.cpp)

add_executable(move_to_waypoint_action_2 src/move_to_waypoint_action_2.cpp)
add_executable(alignment_and_picture_node src/alignment_and_picture_node.cpp)
add_executable(finalization_action_node src/finalization_action_node.cpp)

ament_target_dependencies(get_plan
    rclcpp
    plansys2_pddl_parser
    plansys2_msgs
    plansys2_domain_expert
    plansys2_planner
    plansys2_problem_expert
)

ament_target_dependencies(get_plan_and_execute
    rclcpp
    plansys2_pddl_parser
    plansys2_msgs
    plansys2_domain_expert
    plansys2_planner
    plansys2_problem_expert
    plansys2_executor
    rclcpp_action
)

ament_target_dependencies(rotation_action_node
    rclcpp
    rclcpp_action
    plansys2_executor
    plansys2_msgs
    geometry_msgs
    nav2_msgs
    aruco_opencv_msgs
    marker_service_pkg
)
ament_target_dependencies(move_to_waypoint_action
    rclcpp
    rclcpp_action
    plansys2_executor
    plansys2_msgs
    geometry_msgs
    nav2_msgs
)

ament_target_dependencies(check_end_action_node
    rclcpp
    rclcpp_action
    plansys2_executor
    plansys2_msgs
)

ament_target_dependencies(finalization_action_node
    rclcpp
    rclcpp_action
    plansys2_executor
    plansys2_msgs
)

ament_target_dependencies(move_to_waypoint_action_2
    rclcpp
    rclcpp_action
    plansys2_executor
    plansys2_msgs
    geometry_msgs
    nav2_msgs
    marker_service_pkg
    cv_bridge
    aruco_opencv_msgs
)

ament_target_dependencies(alignment_and_picture_node
    rclcpp
    rclcpp_action
    plansys2_executor
    plansys2_msgs
    geometry_msgs
    nav2_msgs
    sensor_msgs
    marker_service_pkg
    aruco_opencv_msgs
    cv_bridge
)

target_link_libraries(alignment_and_picture_node ${OpenCV_LIBS})
target_link_libraries(move_to_waypoint_action_2 ${OpenCV_LIBS})

install(DIRECTORY
  domain
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  get_plan get_plan_and_execute rotation_action_node move_to_waypoint_action check_end_action_node move_to_waypoint_action_2 alignment_and_picture_node finalization_action_node
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
