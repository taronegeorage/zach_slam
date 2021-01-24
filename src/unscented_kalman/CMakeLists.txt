
#find_package(PCL REQUIRED)
SET(EIGEN_INC_DIR /usr/local/include/eigen3)
#find_package(OpenCV 3.4.1 REQUIRED)

include_directories(
	${EIGEN_INC_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(unscented main.cpp)
#target_link_libraries(unscented
#    ekf_slam_lib
#)
