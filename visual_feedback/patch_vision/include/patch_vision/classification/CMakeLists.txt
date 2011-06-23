rosbuild_add_library(${PROJECT_NAME}_classification classifier.cpp classifiers_common.cpp)
target_link_libraries(${PROJECT_NAME}_classification ${PROJECT_NAME}_slicing ${PROJECT_NAME}_extraction)
