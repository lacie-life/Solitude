cmake_minimum_required(VERSION 3.13)  
project(loop_detection)               
set(CMAKE_CXX_STANDARD 14)          

file(GLOB SOURCE_FILES *.cpp)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(loop_detection ${SOURCE_FILES})
