# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(TrajectoryFitter)

# Use the C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the OpenCV library
# This command looks for the configuration files installed by libopencv-dev
find_package(OpenCV REQUIRED)
# Add OpenCV's include directories to our project
include_directories(${OpenCV_INCLUDE_DIRS})

# Find the Ceres Solver library
# This command looks for the configuration files installed by libceres-dev
find_package(Ceres REQUIRED)
# Add Ceres's include directories
include_directories(${CERES_INCLUDE_DIRS})

# Define the executable target
# This tells CMake to create an executable named "trajectory_fitter" from main.cpp
add_executable(trajectory_fitter main.cpp)

# Link the libraries to our executable
# This is the crucial step that connects your code to the compiled library files
target_link_libraries(trajectory_fitter 
    PRIVATE
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

# Optional: A message to confirm libraries were found
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "Ceres libraries: ${CERES_LIBRARIES}")