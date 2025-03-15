cmake_minimum_required(VERSION 3.10)

# Project name
project(gameboyEmu VERSION 1.0)

# Enable C++17 (or later if needed)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")


add_subdirectory(SDL EXCLUDE_FROM_ALL)


find_package(SDL3 REQUIRED)

file( GLOB_RECURSE SOURCE_FILES lib/*.cpp)

add_executable(${CMAKE_PROJECT_NAME} main.cpp ${SOURCE_FILES})

file(GLOB INCLUDE_FILES include/*.h)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    include)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE SDL3::SDL3)

