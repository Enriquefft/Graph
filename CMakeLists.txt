cmake_minimum_required(VERSION 3.17)
#cmake_minimum_required(VERSION 3.24)

project(Graph VERSION 1.0)


SET(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wshadow -Wdouble-promotion -Wconversion -Wno-string-compare -ferror-limit=1 -g")

set (SOURCES "src/test.cpp")

file(GLOB cpp_sources "src/*.cpp")
list(APPEND SOURCES ${cpp_sources})

include_directories("src")
add_executable(Graph ${SOURCES})

target_link_libraries(Graph "/home/enrique/Documents/POO2/utils/libutils.a")
