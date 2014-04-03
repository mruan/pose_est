cmake_minimum_required (VERSION 2.6)
project (pcd_loader)

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

#add_subdirectory(tests)
add_subdirectory(src)