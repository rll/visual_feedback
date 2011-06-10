rosbuild_add_executable(make_featuremap make_featuremap.cpp)
rosbuild_link_boost(make_featuremap program_options)
target_link_libraries(make_featuremap ${PROJECT_NAME}_extraction ${PROJECT_NAME}_slicing)

