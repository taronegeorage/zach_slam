cmake_minimum_required(VERSION 2.6)
project(zach_slam)
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

add_subdirectory(src/ekf_slam)
add_subdirectory(src/unscented_kalman)
