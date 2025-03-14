set(MOVEIT_LIB_NAME moveit_move_group_interface)


# TODO (ddengster) : port wrap_python_move_group
add_library(${MOVEIT_LIB_NAME}_python SHARED src/wrap_python_move_group.cpp)
target_include_directories(${MOVEIT_LIB_NAME}_python
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${eigenpy_LIBRARIES} ${PYTHON_LIBRARIES} ${LIBS} ${Boost_LIBRARIES} moveit_py_bindings_tools) 
  # /opt/ros/humble/lib/librosidl_typesupport_cpp.so /opt/ros/humble/lib/libmoveit_msgs__rosidl_typesupport_cpp.so)
target_link_directories(${MOVEIT_LIB_NAME}_python
  PUBLIC
  /opt/ros/humble/lib
)

set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_move_group_interface PREFIX "")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin")
#if(WIN32)
#  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
#endif()
#ament_target_dependencies(${MOVEIT_LIB_NAME}_python
#  rclcpp
#  moveit_ros_planning_interface
#)


install(TARGETS ${MOVEIT_LIB_NAME}_python
  EXPORT ${MOVEIT_LIB_NAME}_python
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

