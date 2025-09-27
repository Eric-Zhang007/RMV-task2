cmake_minimum_required(VERSION 3.10)

project(TrajectoryFitter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(trajectory_fitter main.cpp)

target_link_libraries(trajectory_fitter 
    PRIVATE
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "Ceres libraries: ${CERES_LIBRARIES}")
