cmake_minimum_required(VERSION 2.8.3)
project(reuleaux)


find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  octomap_server
  tf2
  rviz_visual_tools
)


 add_message_files(
   FILES
   PointArray.msg
   WorkSpace.msg
   PointColor.msg
   WsColor.msg
 )


 generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs
 )


catkin_package(

  INCLUDE_DIRS
    include
  LIBRARIES
    sphere_discretization
    kinematics
    roscpp
  CATKIN_DEPENDS message_runtime 
)




include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR})



 add_library(sphere_discretization
   src/sphere_discretization.cpp
   )

 add_library(kinematics
   src/kinematics.cpp
   )


 target_link_libraries(sphere_discretization 
   ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
 )
target_link_libraries(kinematics
   ${catkin_LIBRARIES}
 )



 install(TARGETS sphere_discretization kinematics
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )





add_executable(show_sphere src/show_sphere.cpp)
add_dependencies(show_sphere ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_sphere ${catkin_LIBRARIES})

add_executable(show_cone src/show_cone.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_cone ${catkin_LIBRARIES})

add_executable(show_sphere_and_arrow src/show_sphere_and_arrow.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_sphere_and_arrow ${catkin_LIBRARIES})

add_executable(show_sphere_and_arrow_color src/show_sphere_and_arrow_color.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_sphere_and_arrow_color ${catkin_LIBRARIES})

add_executable(show_arrow src/show_arrow.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_arrow ${catkin_LIBRARIES})

add_executable(show_pc src/show_pc.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_pc ${catkin_LIBRARIES})

add_executable(rand_pos_generator src/rand_pos_generator.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rand_pos_generator ${catkin_LIBRARIES})

add_executable(rand_orientation_generator src/rand_orientation_generator.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rand_orientation_generator ${catkin_LIBRARIES})

add_executable(create_sphere_and_show src/create_sphere_and_show.cpp)
target_link_libraries(create_sphere_and_show sphere_discretization ${catkin_LIBRARIES})

add_executable(create_discretized_sphere src/create_discretized_sphere.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_discretized_sphere sphere_discretization ${catkin_LIBRARIES})

add_executable(show_discretized_sphere src/show_discretized_sphere.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_discretized_sphere sphere_discretization ${catkin_LIBRARIES})

add_executable(create_pose_and_show src/create_pose_and_show.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_pose_and_show sphere_discretization kinematics ${catkin_LIBRARIES})

#add_executable(create_sphere_and_arrow src/create_sphere_and_arrow.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(create_sphere_and_arrow sphere_discretization kinematics ${catkin_LIBRARIES})

add_executable(create_sphere_and_arrow_color src/create_sphere_and_arrow_color.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_sphere_and_arrow_color sphere_discretization kinematics -lhdf5 -lhdf5_cpp ${catkin_LIBRARIES})

add_executable(create_maps src/create_maps.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_maps sphere_discretization kinematics -lhdf5 -lhdf5_cpp ${catkin_LIBRARIES})


add_executable(create_capability_map src/create_capability_map.cpp)
add_dependencies(show_cone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_capability_map sphere_discretization kinematics -lhdf5 -lhdf5_cpp ${catkin_LIBRARIES})

