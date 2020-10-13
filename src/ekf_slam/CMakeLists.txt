
#find_package(PCL REQUIRED)
SET(EIGEN_INC_DIR /usr/local/include/eigen3)
include_directories(
	${EIGEN_INC_DIR}
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

add_executable(efk_slam main.cpp)
#target_link_libraries(ekf_slam
#	ekf_slam_lib
#)
