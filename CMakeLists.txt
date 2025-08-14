cmake_minimum_required(VERSION 3.15)
project(moveit_planning)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-error=unused-value)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  rosparam_shortcuts
  geometry_msgs
  tf  
  tf2_eigen
  tf2_geometry_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
  CATKIN_DEPENDS
    tf2
    tf2_geometry_msgs
    tf2_eigen
)

###############
#  Libraries  #
###############

add_library(moveit_utils src/utils.cpp)
target_link_libraries(moveit_utils PUBLIC ${catkin_LIBRARIES})

add_library(approch_utils src/approch_utils.cpp src/grasp_utils.cpp src/retreat_utils.cpp src/solvers_utils.cpp src/config_utils.cpp src/visualization_utils.cpp src/pipeline.cpp) 
target_link_libraries(approch_utils PUBLIC ${catkin_LIBRARIES})

###############
# Executables #
###############

add_executable(moncube nodes/grasp/moncube.cpp)
target_link_libraries(moncube ${catkin_LIBRARIES})
add_dependencies(moncube ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(main nodes/grasp/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} approch_utils)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(interactive_grasp nodes/grasp/interactive_grasp.cpp)
target_link_libraries(interactive_grasp ${catkin_LIBRARIES} moveit_utils)
add_dependencies(interactive_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(interactive_spawn_cube nodes/spawn/interactive_spawn_cube.cpp)
target_link_libraries(interactive_spawn_cube ${catkin_LIBRARIES})
add_dependencies(interactive_spawn_cube ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pick_place_demo nodes/grasp/pick_place_demo.cpp)
target_link_libraries(pick_place_demo ${catkin_LIBRARIES})
add_dependencies(pick_place_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(interactive_place nodes/grasp/interactive_place.cpp)
target_link_libraries(interactive_place ${catkin_LIBRARIES})

add_executable(testmain nodes/grasp/testmain.cpp)
target_link_libraries(testmain ${catkin_LIBRARIES} approch_utils)
add_dependencies(testmain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(face_colors nodes/grasp/face_colors.cpp)
target_link_libraries(face_colors ${catkin_LIBRARIES})

add_executable(orient_and_place_cube nodes/grasp/orient_and_place_cube.cpp)
target_link_libraries(orient_and_place_cube ${catkin_LIBRARIES} moveit_utils)

add_executable(spawn_scene nodes/spawn/spawn_scene.cpp)
target_link_libraries(spawn_scene ${catkin_LIBRARIES})

###############
#   Install   #
###############

install(TARGETS 
  pick_place_demo
  moncube
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
