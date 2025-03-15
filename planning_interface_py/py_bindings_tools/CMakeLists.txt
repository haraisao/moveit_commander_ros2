set(MOVEIT_LIB_NAME moveit_py_bindings_tools)


add_library(${MOVEIT_LIB_NAME} SHARED src/roscpp_initializer.cpp)
ament_target_dependencies(${MOVEIT_LIB_NAME} rclcpp
  moveit_ros_planning_interface
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  EXPORT ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION share
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

add_library(${MOVEIT_LIB_NAME}_python SHARED src/wrap_python_roscpp_initializer.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "lib")
if(WIN32)
  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif()

install(TARGETS ${MOVEIT_LIB_NAME}_python
  EXPORT ${MOVEIT_LIB_NAME}_python
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

