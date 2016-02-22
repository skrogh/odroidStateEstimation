cmake_minimum_required(VERSION 2.8.7)
project(state_estimation_odroid)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system chrono)

find_library(imu_lib NAMES imu_lib REQUIRED
	PATHS ${PROJECT_SOURCE_DIR}/external/imuDriver/lib
)

find_library(imu_lib NAMES ekf_lib REQUIRED
	PATHS ${PROJECT_SOURCE_DIR}/external/msf_ekf/lib
)

find_library(imu_lib NAMES ekf_lib REQUIRED
	PATHS ${PROJECT_SOURCE_DIR}/external/msf_ekf/lib
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

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/external
 )


# build test program
add_executable(main_test src/test/main_test.cpp)
#target_link_libraries(main_test imu_lib ekf_lib lsd_lib)
target_link_libraries(main_test ${imu_lib} ${Boost_LIBRARIES})