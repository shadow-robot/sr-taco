cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#find_package(OpenRAVE)
#-- Get the openrave share directory
#EXEC_PROGRAM("openrave-config --share-dir" OUTPUT_VARIABLE OpenRAVE_SHARE_DIR)
#include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )

#generate actionlib messages
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

#rosbuild_add_executable(analyse_moving_object src/analyse_moving_object.cpp src/prediction_model.cpp)
#SET( CMAKE_CXX_FLAGS "-D IKFAST_NO_MAIN -D IKFAST_CLIBRARY -D IKFAST_NAMESPACE=ik_fast")
#rosbuild_add_executable(visual_servoing src/visual_servoing.cpp)
##rosbuild_add_openmp_flags(visual_servoing)
#target_link_libraries(visual_servoing ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

rosbuild_add_executable(visual_servoing_moveit src/visual_servoing_moveit.cpp src/visual_servoing_action_server.cpp)
