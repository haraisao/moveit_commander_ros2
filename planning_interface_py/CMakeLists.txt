cmake_minimum_required(VERSION 3.22)
project(moveit_ros_planning_interface_py)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if($ENV{ROS_DISTRO} STREQUAL "humble")
    add_compile_options(-Wall -Wextra -Wpedantic -DROS_HUMBLE)
  else()
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(eigenpy REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

ament_python_install_package(moveit_ros_planning_interface_py)

set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
)

set(THIS_PACKAGE_LIBRARIES
  moveit_py_bindings_tools
  moveit_py_bindings_tools_python
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_warehouse
  moveit_ros_move_group
  moveit_ros_planning_interface
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  rclcpp
  rclcpp_action
  Eigen3
  Boost
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${Python3_NumPy_INCLUDE_DIRS})

add_subdirectory(py_bindings_tools)
add_subdirectory(planning_scene_interface)
add_subdirectory(move_group_interface)
add_subdirectory(robot_interface)

#############
## TESTING ##
#############

#if(BUILD_TESTING)
#  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ros_testing REQUIRED)

  # TODO (vatanaksoytezer): Enable once this test is not flaky
  # Move group interface cpp ompl constrained planning integration test
  # ament_add_gtest_executable(move_group_ompl_constraints_test
  #     test/move_group_ompl_constraints_test.cpp
  # )

  # target_link_libraries(move_group_ompl_constraints_test ${THIS_PACKAGE_LIBRARIES})
  # ament_target_dependencies(move_group_ompl_constraints_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  # add_ros_test(test/launch/move_group_ompl_constraints.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # end move group interface cpp ompl constrained planning integration test

#endif()

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#install(DIRECTORY python/ DESTINATION ${PYTHON_LIB_DESTINATION})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
#  set(ament_cmake_copyright_FOUND TRUE)
#  set(ament_cmake_cpplint_FOUND TRUE)
#  set(ament_cmake_flake8_FOUND TRUE)
#  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
