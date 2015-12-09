cmake_minimum_required(VERSION 2.8.3)
project(baxter_asu)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_description
  roscpp
#  cv_bridge
#  ar_track_alvar
#  ar_track_alvar_msgs
)

find_package( gazebo REQUIRED )
find_package( Boost REQUIRED system filesystem regex )
find_package( Protobuf REQUIRED )
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )


# Add our cmake find modules for odd-packages with no CMake or unusual locations
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules )

# For calibration
find_package( alvar REQUIRED )
include_directories( ${ALVAR_INCLUDE_DIRS} )
message( STATUS "!!!ALVAR INCLUDE DIRS!!!:${ALVAR_INCLUDE_DIRS}" )

# For interloper
find_package( ach REQUIRED )
find_package( sns REQUIRED )
find_package( amino REQUIRED )
find_package( PkgConfig )
pkg_check_modules( EIGEN REQUIRED eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

include_directories( ${ACH_INCLUDE_DIRS} )
include_directories( ${SNS_INCLUDE_DIRS} )
include_directories( ${AMINO_INCLUDE_DIRS} )

find_package( PythonLibs REQUIRED )
include_directories( ${PYTHON_INCLUDE_DIRS} )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_asu
  CATKIN_DEPENDS baxter_description roscpp
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
#add_executable(view_kinect_mode src/view_kinect_node.cpp)
#target_link_libraries( view_kinect_mode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

#add_executable( calibration_kinect_hand_camera src/calibration_kinect_hand_camera.cpp)
#target_link_libraries( calibration_kinect_hand_camera baseMarkerDetector ${PCL_LIBRARIES} ${ALVAR_LIBRARIES}  ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )


#add_library( baseMarkerDetector src/baseMarkerDetector/baseMarkerDetector.cpp )
#target_link_libraries( baseMarkerDetector ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} )


include_directories( $ENV{HOME}/local/include/golems )
link_directories( $ENV{HOME}/local/lib )

add_library( interloper SHARED src/interloper/interloper.cpp )
target_link_libraries( interloper tkunz bimanual_msgs ${ACH_LIBRARIES} ${SNS_LIBRARIES} ${AMINO_LIBRARIES} ${roscpp_LIBRARIES} )

set_target_properties( interloper PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/python2.7/dist-packages  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/python2.7/dist-packages PREFIX "" )

add_executable( test_interloper_generator_gk src/interloper/test_interloper_generator_gk.cpp )
target_link_libraries( test_interloper_generator_gk  bimanual_msgs ${ACH_LIBRARIES} ${SNS_LIBRARIES} ${AMINO_LIBRARIES} )

