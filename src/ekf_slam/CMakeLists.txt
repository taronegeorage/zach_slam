
#find_package(PCL REQUIRED)
SET(EIGEN_INC_DIR /usr/local/include/eigen3)
find_package(OpenCV 3.4.1 REQUIRED)

include_directories(
	${EIGEN_INC_DIR}
    ${PROJECT_SOURCE_DIR}/include
)
add_library(ekf_slam_lib
    extended_kalman_filter.cpp
)

target_link_libraries(ekf_slam_lib
	${OpenCV_LIBS}
)

#target_link_libraries(${PROJECT_NAME}_lib
#    ${EIGEN3_LIBS}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#)

add_executable(ekf_slam main.cpp)
target_link_libraries(ekf_slam
    ekf_slam_lib
)
