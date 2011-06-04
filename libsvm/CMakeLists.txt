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

#Making SVM library
rosbuild_add_library(svm include/libsvm/svm.cpp)
rosbuild_add_executable(svm-train include/libsvm/svm-train.c)
target_link_libraries(svm-train svm)
rosbuild_add_executable(svm-predict include/libsvm/svm-predict.c)
target_link_libraries(svm-predict svm)
rosbuild_add_executable(svm-scale include/libsvm/svm-scale.c)
target_link_libraries(svm-scale svm)
