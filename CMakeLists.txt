cmake_minimum_required(VERSION 2.8.3)
project(baxter_asu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_description
  roscpp
)

find_package( gazebo REQUIRED )
find_package( Boost REQUIRED system filesystem regex )
find_package( Protobuf REQUIRED )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_asu
#  CATKIN_DEPENDS baxter_description roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library( hello_world_plugin src/hello_world_plugin.cc )
target_link_libraries( hello_world_plugin ${GAZEBO_LIBRARIES} )


add_library( tutorial_push_model src/tutorial_push_model.cc )
target_link_libraries( tutorial_push_model ${GAZEBO_LIBRARIES} )


add_library( tutorial_factory src/tutorial_factory.cc )
target_link_libraries( tutorial_factory ${GAZEBO_LIBRARIES} )


add_library( tutorial_gravity src/tutorial_gravity.cc )
target_link_libraries( tutorial_gravity ${GAZEBO_LIBRARIES} )


add_executable( tutorial_transport src/tutorial_transport.cc )
target_link_libraries( tutorial_transport ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES} pthread )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(baxter_asu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(baxter_asu_node src/baxter_asu_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(baxter_asu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(baxter_asu_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/an_1.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables and/or libraries for installation
# install(TARGETS baxter_asu baxter_asu_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baxter_asu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
