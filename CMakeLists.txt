cmake_minimum_required(VERSION 2.8.7)
project(state_estimation_odroid)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d)
find_package(Boost REQUIRED COMPONENTS thread system)

find_library(imu_lib NAMES imu_lib REQUIRED
	PATHS ${PROJECT_SOURCE_DIR}/external/imuDriver/lib
)

find_library(ekf_lib NAMES ekf_lib REQUIRED
	PATHS ${PROJECT_SOURCE_DIR}/external/msf_ekf/lib
)

find_library(lsdslam NAMES lsdslam REQUIRED
	PATHS ${PROJECT_SOURCE_DIR}/external/lsd_slam/lsd_slam_core/lib
)


# SSE flags
#add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
#set(CMAKE_CXX_FLAGS
#   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -O2 -march=native  -std=c++0x"
#)

# NEON flags
add_definitions("-DENABLE_NEON")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -O2 -march=native -mfpu=neon -std=c++0x"
) 

set(LSD_DIR ${PROJECT_SOURCE_DIR}/external/lsd_slam/lsd_slam_core)
set(IMU_DIR ${PROJECT_SOURCE_DIR}/external/imuDriver)
set(MSF_DIR ${PROJECT_SOURCE_DIR}/external/msf_ekf)
set(BUFS_DIR ${PROJECT_SOURCE_DIR}/external/concurrentBuffers)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${LSD_DIR}/thirdparty/Sophus
  ${LSD_DIR}/src
  ${IMU_DIR}/src
  ${MSF_DIR}/src
  ${BUFS_DIR}/src
 )


# build test program
add_executable(main_test src/test/main_test.cpp)
target_link_libraries(main_test ${imu_lib} ${ekf_lib} ${lsdslam} ${FABMAP_LIB} g2o_core g2o_stuff cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(main_test ${imu_lib} ${Boost_LIBRARIES})
