cmake_minimum_required (VERSION 2.6)
project(StereoTests)

# Collect all sources
file(GLOB_RECURSE EKFHEXA_SOURCE_FILES "src/*.cpp" "src/*.h" "src/*.inl" "../objectsMap/src/utils/gui/graph2d*")

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(src)

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

# Loading 3rd party libraries.
find_package(OpenCV 3 REQUIRED core imgproc highgui)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
LINK_DIRECTORIES(${OpenCV_LIB_DIR})


find_package(catkin COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	std_srvs
	message_generation
)

if(catkin_FOUND)
	add_definitions(-D_HAS_ROS_LIBRARIES_)
endif()

# Add Executable
add_executable(ekfHexa ${EKFHEXA_SOURCE_FILES})

target_include_directories(ekfHexa PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_include_directories(ekfHexa PUBLIC ${PROJECT_SOURCE_DIR}/../lib/DroneApplications/src/code)
target_include_directories(ekfHexa PUBLIC ${PROJECT_SOURCE_DIR}/../lib/bovil/src/code)
target_include_directories(ekfHexa PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(ekfHexa PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/StereoLib/src)

target_link_libraries(ekfHexa LINK_PUBLIC DroneApplications)
target_link_libraries(ekfHexa LINK_PUBLIC BOViL)
target_link_libraries(ekfHexa LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(ekfHexa LINK_PUBLIC StereoLib)