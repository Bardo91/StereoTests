cmake_minimum_required (VERSION 2.6)
project(StereoTests)

# Adding cmake module paths.
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} /usr/lib/cmake /usr/local/lib/cmake)

# Project directory
add_subdirectory(projects)
