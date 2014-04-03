
#add_executable(test_loadpcd test_loadpcd.cpp pcd_utils.cpp label_utils.cpp)
#target_link_libraries(test_loadpcd ${PCL_LIBRARIES})

add_executable(compute_feature compute_feature.cpp pcd_utils.cpp label_utils.cpp)
target_link_libraries(compute_feature ${PCL_LIBRARIES})
