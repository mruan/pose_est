cmake_minimum_required (VERSION 2.6)
project (pcd_loader)

# we need FindFLANN.cmake 
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wno-deprecated-declarations")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})#${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL 1.7 REQUIRED PATHS /usr NO_DEFAULT_PATH)

include_directories(
  ${PCL_INCLUDE_DIRS}
  .)

link_directories(
  /usr/local/lib/
  ${PCL_LIBRARY_DIRS}
  )
add_definitions(${PCL_DEFINITIONS})


find_package(HDF5)

if(HDF5_FOUND)

  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(build_model src/build_model.cpp)
  target_link_libraries(build_model ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY})

  add_executable(mt_loadtestdata src/modtest_loadtestdata.cpp)
  target_link_libraries(mt_loadtestdata ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY})

  add_executable(mt_loadtraindata src/modtest_loadtraindata.cpp)
  target_link_libraries(mt_loadtraindata ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY})
endif(HDF5_FOUND)

#add_subdirectory(tests)
add_subdirectory(src)