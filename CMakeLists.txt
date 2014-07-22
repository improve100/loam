cmake_minimum_required(VERSION 2.8.3)
project(loam)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES loam
  CATKIN_DEPENDS roscpp
  DEPENDS libpcl_all
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${PCL_INCLUDE_DIRS})

## Declare a cpp library
add_library(loam
  src/ScanRegistration.cpp
  src/LaserOdometry.cpp
)

## Declare a cpp executable
add_executable(scan_registration src/ScanRegistrationNode.cpp)
add_executable(laser_odometry src/LaserOdometryNode.cpp)
add_executable(trigger src/TriggerNode.cpp)
add_executable(odom_old src/odom_old.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(loam_node loam_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(loam
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

target_link_libraries(scan_registration
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   loam
)

target_link_libraries(laser_odometry
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   loam
)

target_link_libraries(trigger
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

target_link_libraries(odom_old ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS loam
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_loam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
