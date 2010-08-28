cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} include/SocksDemo/GripPointDetector.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/SocksDemo/SockStateDetector.cpp)
#target_link_libraries(${PROJECT_NAME} include/SocksDemo/build/Demo/GNU-Linux-x86)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(cpp_node include/cpp_node.cpp)
#rosbuild_add_compile_flags(cpp_node "-lsocks -L/home/stephen_sandbox/visual_feedback/image_processor/lib")
#rosbuild_add_library(${PROJECT_NAME} include/SocksDemo/GripPointDetector.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/SocksDemo/SockStateDetector.cpp)
rosbuild_add_link_flags(cpp_node "-L${PROJECT_SOURCE_DIR}/lib")
#target_link_libraries(cpp_node socks)
