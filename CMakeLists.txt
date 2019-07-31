cmake_minimum_required(VERSION 2.6.0)

project(sweep_filter)

add_definitions(-std=c++11)
add_definitions("-DENABLE_SSE")

SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

find_package(OpenMP REQUIRED)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set (HEADER
)

set (SRC
    src/main.cpp
)

# Find catkin and Packages
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros pcl_conversions std_msgs sensor_msgs)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp pcl_ros std_msgs sensor_msgs
               INCLUDE_DIRS include)
#               LIBRARIES ${PROJECT_NAME})
#               DEPENDS OpenCV Sophus


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable (sweep_filter ${SRC})
target_link_libraries (sweep_filter
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${catkin_LIBRARIES}
#   ${Sophus_LIBS}
#   ${DBoW2_LIBS}
#   ${DLib_LIBS}
#   ${PCL_LIBRARIES}
#   ${CHOLMOD_LIBRARIES}
##   ${CSPARSE_LIBRARY}
#   ${IRPLIB_G2O}
)

