cmake_minimum_required(VERSION 2.8.3)

project(base_placement_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  pluginlib
  roscpp
  rqt_gui
  rqt_gui_cpp
  rviz
  visualization_msgs
  tf
  tf_conversions
  map_creator
  pcl_ros
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})


add_definitions(-DQT_NO_KEYWORDS)


add_library(create_marker  src/create_marker.cpp)
target_link_libraries(create_marker  ${catkin_LIBRARIES} )

set(base_placement_plugin_SRCS
  src/add_way_point.cpp
  src/point_tree_item.cpp
  src/point_tree_model.cpp
  src/widgets/base_placement_widget.cpp
  src/place_base.cpp
  src/add_robot_base.cpp

  
  ${MOC_FILES}
)

set(base_placement_plugin_HDRS
  include/base_placement_plugin/add_way_point.h
  include/base_placement_plugin/point_tree_item.h
  include/base_placement_plugin/point_tree_model.h
  include/base_placement_plugin/widgets/base_placement_widget.h
  include/base_placement_plugin/place_base.h
  include/base_placement_plugin/add_robot_base.h

  
)

set(base_placement_plugin_UIS
  src/widgets/base_placement_widget.ui
)

set(base_placement_plugin_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${base_placement_plugin_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  interactive_markers
  pluginlib
  roscpp
  rqt_gui
  rviz
  rqt_gui_cpp
  visualization_msgs
  map_creator
)

qt4_wrap_cpp(base_placement_plugin_MOCS ${base_placement_plugin_HDRS} )
qt4_wrap_ui(base_placement_plugin_UIS_H ${base_placement_plugin_UIS} )

include_directories(${base_placement_plugin_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${base_placement_plugin_SRCS} ${base_placement_plugin_MOCS} ${base_placement_plugin_UIS_H}) # ${base_placement_plugin_UIS_H}
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}  yaml-cpp -lhdf5 -lhdf5_cpp  create_marker)
## target_link_libraries(${PROJECT_NAME} yaml-cpp)



find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/base_placement_plugin
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

