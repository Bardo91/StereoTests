cmake_minimum_required (VERSION 2.6)
project(StereoTests)
# Project dependencies
add_subdirectory(lib/cjson/src)
add_subdirectory(lib/libsvmpp/src)
add_subdirectory(lib/DroneApplications/src)
add_subdirectory(lib/DroneApplications/lib/bovil/src)
add_subdirectory(/lib/bovil/src)


# Project directory
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_subdirectory(projects)
