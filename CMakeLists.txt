cmake_minimum_required(VERSION 2.6)
project(zach_slam)
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

#find_package(PCL REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)
#add_library(ekf_slam_lib SHARED
#    src/ekf_slam/main.cpp
#)

#target_link_libraries(${PROJECT_NAME}_lib
#    ${EIGEN3_LIBS}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#)

add_executable(efk_slam src/ekf_slam/main.cpp)
#target_link_libraries(ekf_slam
#	ekf_slam_lib
#)
