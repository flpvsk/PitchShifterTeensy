cmake_minimum_required(VERSION 3.10)

# set the project name
project(PitchShifterTeensy VERSION 1.0)

set (CMAKE_CXX_STANDARD 20)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# add the executable
add_executable(PitchShifterTest main_test.cc PitchShifterAlg.cc)

