cmake_minimum_required (VERSION 2.6)
project(StereoTests)
# Project dependencies
MESSAGE(STATUS "---------------	Preparing cjson library	---------------")
add_subdirectory(lib/cjson/src)
MESSAGE(STATUS "---------------	Preparing libsvm library	---------------")
add_subdirectory(lib/libsvmpp/src)
MESSAGE(STATUS "---------------	Preparing Drone Applications library	---------------")
add_subdirectory(lib/DroneApplications/src)
MESSAGE(STATUS "---------------	Preparing bovil library	---------------")
add_subdirectory(lib/bovil/src)
MESSAGE(STATUS "---------------	Preparing StereoLib library	---------------")
add_subdirectory(lib/StereoLib/src)


# Project directory
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_subdirectory(projects)
