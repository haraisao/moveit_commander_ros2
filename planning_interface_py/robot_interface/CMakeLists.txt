set(MOVEIT_LIB_NAME moveit_robot_interface)

#include_directories("/usr/include/python3.10")
add_library(${MOVEIT_LIB_NAME}_python SHARED src/wrap_python_robot_interface.cpp)

target_link_libraries(${MOVEIT_LIB_NAME}_python ${PYTHON_LIBRARIES} ${Boost_LIBRARIES}
  moveit_common_planning_interface_objects moveit_py_bindings_tools
)
target_link_directories(${MOVEIT_LIB_NAME}_python PUBLIC /opt/ros/$ENV{ROS_DISTRO}/lib)
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_robot_interface PREFIX "")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "lib/")
if(WIN32)
  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif()

install(TARGETS ${MOVEIT_LIB_NAME}_python
  EXPORT ${MOVEIT_LIB_NAME}_python
  DESTINATION  ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
