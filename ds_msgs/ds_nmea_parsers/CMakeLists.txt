cmake_minimum_required(VERSION 2.8.3)
project(ds_nmea_parsers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        ds_nmea_msgs
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ds_nmea_parsers
   CATKIN_DEPENDS ds_nmea_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/Hdt.h
  src/${PROJECT_NAME}/Hdt.cpp

  include/${PROJECT_NAME}/Gll.h
  src/${PROJECT_NAME}/Gll.cpp

  include/${PROJECT_NAME}/Vtg.h
  src/${PROJECT_NAME}/Vtg.cpp

  include/${PROJECT_NAME}/Gga.h
  src/${PROJECT_NAME}/Gga.cpp

  include/${PROJECT_NAME}/Zda.h
  src/${PROJECT_NAME}/Zda.cpp

  include/${PROJECT_NAME}/PixseAlgsts.h
  src/${PROJECT_NAME}/PixseAlgsts.cpp

  include/${PROJECT_NAME}/PixseAtitud.h
  src/${PROJECT_NAME}/PixseAtitud.cpp

  include/${PROJECT_NAME}/PixseHeave.h
  src/${PROJECT_NAME}/PixseHeave.cpp

  include/${PROJECT_NAME}/PixseHspos.h
  src/${PROJECT_NAME}/PixseHspos.cpp

  include/${PROJECT_NAME}/PixseHtsts.h
  src/${PROJECT_NAME}/PixseHtsts.cpp

  include/${PROJECT_NAME}/PixsePositi.h
  src/${PROJECT_NAME}/PixsePositi.cpp

  include/${PROJECT_NAME}/PixseSpeed.h
  src/${PROJECT_NAME}/PixseSpeed.cpp

  include/${PROJECT_NAME}/PixseStatus.h
  src/${PROJECT_NAME}/PixseStatus.cpp

  include/${PROJECT_NAME}/PixseStdhrp.h
  src/${PROJECT_NAME}/PixseStdhrp.cpp

  include/${PROJECT_NAME}/PixseStdpos.h
  src/${PROJECT_NAME}/PixseStdpos.cpp

  include/${PROJECT_NAME}/PixseStdspd.h
  src/${PROJECT_NAME}/PixseStdspd.cpp

  include/${PROJECT_NAME}/PixseUtmwgs.h
  src/${PROJECT_NAME}/PixseUtmwgs.cpp

  include/${PROJECT_NAME}/util.h
  src/${PROJECT_NAME}/util.cpp

  include/${PROJECT_NAME}/Inf.h
  src/${PROJECT_NAME}/Inf.cpp

  include/${PROJECT_NAME}/Tro.h
  src/${PROJECT_NAME}/Tro.cpp

  include/${PROJECT_NAME}/PixseFogest.h
  src/${PROJECT_NAME}/PixseFogest.cpp

  include/${PROJECT_NAME}/PixseAccest.h
  src/${PROJECT_NAME}/PixseAccest.cpp

  include/${PROJECT_NAME}/PixseConfig1Int.h
  src/${PROJECT_NAME}/PixseConfig1Int.cpp

  include/${PROJECT_NAME}/PixseConfig2Int.h
  src/${PROJECT_NAME}/PixseConfig2Int.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ds_nmea_parsers_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ds_nmea_parsers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_hehdt src/test/test_hehdt.cpp)
    target_link_libraries(test_hehdt ${PROJECT_NAME})

    catkin_add_gtest(test_gga src/test/test_gga.cpp)
    target_link_libraries(test_gga ${PROJECT_NAME})

    catkin_add_gtest(test_gll src/test/test_gll.cpp)
    target_link_libraries(test_gll ${PROJECT_NAME})

    catkin_add_gtest(test_vtg src/test/test_vtg.cpp)
    target_link_libraries(test_vtg ${PROJECT_NAME})

    catkin_add_gtest(test_zda src/test/test_zda.cpp)
    target_link_libraries(test_zda ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_atitud src/test/test_pixse_atitud.cpp)
    target_link_libraries(test_pixse_atitud ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_positi src/test/test_pixse_positi.cpp)
    target_link_libraries(test_pixse_positi ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_algsts src/test/test_pixse_algsts.cpp)
    target_link_libraries(test_pixse_algsts ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_htsts src/test/test_pixse_htsts.cpp)
    target_link_libraries(test_pixse_htsts ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_status src/test/test_pixse_status.cpp)
    target_link_libraries(test_pixse_status ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_heave src/test/test_pixse_heave.cpp)
    target_link_libraries(test_pixse_heave ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_speed src/test/test_pixse_speed.cpp)
    target_link_libraries(test_pixse_speed ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_stdspd src/test/test_pixse_stdspd.cpp)
    target_link_libraries(test_pixse_stdspd ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_stdpos src/test/test_pixse_stdpos.cpp)
    target_link_libraries(test_pixse_stdpos ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_stdhrp src/test/test_pixse_stdhrp.cpp)
    target_link_libraries(test_pixse_stdhrp ${PROJECT_NAME})

    catkin_add_gtest(test_pixse_utmwgs src/test/test_pixse_utmwgs.cpp)
    target_link_libraries(test_pixse_utmwgs ${PROJECT_NAME})

    catkin_add_gtest(test_util src/test/test_util.cpp)
    target_link_libraries(test_util ${PROJECT_NAME})

    catkin_add_gtest(test_tro src/test/test_tro.cpp)
    target_link_libraries(test_tro ${PROJECT_NAME})

    catkin_add_gtest(test_inf src/test/test_inf.cpp)
    target_link_libraries(test_inf ${PROJECT_NAME})
endif(CATKIN_ENABLE_TESTING)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
