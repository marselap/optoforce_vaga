cmake_minimum_required(VERSION 2.8.3)
project(optoforce_vaga)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES optoforce_vaga
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS} include
)


add_library(Tf1 src/Tf1.cpp)
target_link_libraries(Tf1 ${catkin_LIBRARIES})

add_library(median_filter src/median_filter.cpp)
target_link_libraries(median_filter ${catkin_LIBRARIES})


add_executable(optoforce_sensor_node src/optoforce_sensor_node.cpp)
set(OMD_BIN ${PROJECT_SOURCE_DIR}/lib/libOMD.so)

add_executable(force_filter_node src/force_filter_node.cpp)
target_link_libraries(force_filter_node ${catkin_LIBRARIES} median_filter Tf1)
#add_library(libomd 

target_link_libraries(optoforce_sensor_node
   ${catkin_LIBRARIES} ${OMD_BIN}
 )

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
# install(TARGETS optoforce_vaga optoforce_vaga_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_optoforce_vaga.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
