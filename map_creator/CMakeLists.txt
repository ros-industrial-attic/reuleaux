cmake_minimum_required(VERSION 2.8.3)
project(map_creator)


find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  tf2
  rviz_visual_tools
  pcl_ros
)

find_package(octomap REQUIRED)

 add_message_files(
   FILES
   capShape.msg
   capability.msg
   WsSphere.msg
   WorkSpace.msg
   
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
    hdf5_dataset
    roscpp
  CATKIN_DEPENDS message_runtime 
)




include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR} ${PCL_INCLUDE_DIR})



 add_library(sphere_discretization
   src/sphere_discretization.cpp
   )

 add_library(kinematics
   src/kinematics.cpp
   )
 add_library(hdf5_dataset
   src/hdf5_dataset.cpp
   )


 target_link_libraries(sphere_discretization 
   ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARY_DIRS}
 )
target_link_libraries(kinematics
   ${catkin_LIBRARIES}
 )

target_link_libraries(hdf5_dataset
   ${catkin_LIBRARIES}
 )



 install(TARGETS sphere_discretization kinematics hdf5_dataset
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   
 )


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )


add_executable(create_reachability_map src/create_reachability_map.cpp)
add_dependencies(create_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_reachability_map sphere_discretization kinematics -lhdf5 -lhdf5_cpp ${catkin_LIBRARIES})

add_executable(create_inverse_reachability_map src/create_inverse_reachability_map.cpp)
add_dependencies(create_inverse_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_inverse_reachability_map sphere_discretization kinematics -lhdf5 -lhdf5_cpp hdf5_dataset ${catkin_LIBRARIES})

add_executable(create_capability_map src/create_capability_map.cpp)
add_dependencies(create_capability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_capability_map sphere_discretization kinematics -lhdf5 -lhdf5_cpp ${catkin_LIBRARIES})

add_executable(load_reachability_map src/load_reachability_map.cpp)
add_dependencies(load_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_reachability_map -lhdf5 -lhdf5_cpp hdf5_dataset ${catkin_LIBRARIES})

add_executable(load_capability_map src/load_capability_map.cpp)
add_dependencies(load_capability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_capability_map -lhdf5 -lhdf5_cpp hdf5_dataset ${catkin_LIBRARIES})

catkin_add_gtest(utest test/utest.cpp)
target_link_libraries(utest sphere_discretization ${catkin_LIBRARIES})

