cmake_minimum_required(VERSION 2.8.3)
project(world_item_search)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  worldlib
)
find_package(Boost REQUIRED COMPONENTS
  random
  filesystem
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(offline_item_search
  nodes/offline_item_search.cpp
  src/GeoLifeEntry.cpp
  src/OfflineItemSearcher.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(offline_item_search
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS offline_item_search
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
