rosbuild_add_library(${PROJECT_NAME}_extraction descriptor.cpp descriptors_common.cpp feature_io.cpp)
target_link_libraries(${PROJECT_NAME}_extraction ${PROJECT_NAME}_slicing ${PROJECT_NAME}_thirdparty)
