cmake_minimum_required(VERSION 2.8.3)
project(worldlib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  tf2
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS
  random
)
find_package(PkgConfig)

###################################################
## Declare things to be passed to other projects ##
###################################################

## External libraries
set(WORLDLIB_EXTERN_LIBS
  curl
  jsoncpp
  mysqlclient
  yaml-cpp
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp tf2
  DEPENDS Boost ${WORLDLIB_EXTERN_LIBS}
)

###########
## Build ##
###########


## Check for what version of YAML we have
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DYAMLCPP_GT_0_5_0)
endif()

## Specify additional locations of header files
include_directories(include
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Source files for the library
set(WORLDLIB_SOURCE
  # Geometry source files
  src/geometry/Orientation.cpp
  src/geometry/Pose.cpp
  src/geometry/Position.cpp
  # Model source files
  src/model/PersistenceModel.cpp
  src/model/PlacementModel.cpp
  src/model/TaskModel.cpp
  # Remote source files
  src/remote/Client.cpp
  src/remote/HttpClient.cpp
  src/remote/InteractiveWorldModelClient.cpp
  src/remote/Node.cpp
  src/remote/SpatialWorldClient.cpp
  src/remote/SpatialWorldObservation.cpp
  src/remote/SqlClient.cpp
  src/remote/SqlEntity.cpp
  # World source files
  src/world/Item.cpp
  src/world/Object.cpp
  src/world/Observation.cpp
  src/world/Placement.cpp
  src/world/PlacementSurface.cpp
  src/world/PointOfInterest.cpp
  src/world/Room.cpp
  src/world/Surface.cpp
  src/world/World.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${WORLDLIB_SOURCE})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${WORLDLIB_EXTERN_LIBS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
