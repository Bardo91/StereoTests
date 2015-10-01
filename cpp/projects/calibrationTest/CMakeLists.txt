cmake_minimum_required (VERSION 2.6)
project(bar_task)

# Collect all sources
file(GLOB_RECURSE BAR_TASK_SOURCE_FILES "src/*.cpp" "src/*.h" "src/*.inl")

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

macro(IncludeFromEnvironment envVar)
	set(INCLUDE_DIR ${envVar})
	if(DEFINED INCLUDE_DIR)
		include_directories(${INCLUDE_DIR})
	else()
		message(SEND_ERROR "Cant locate eigen include directory. Please define an environment variable with include directory")
	endif()
endmacro()

GroupSources(code)

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} /usr/lib/cmake /usr/local/lib/cmake)
# Loading 3rd party libraries.
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})
include_directories(${OPENCV_LIB_DIR})

# Add Executable
add_executable(calibrationTest ${BAR_TASK_SOURCE_FILES})

target_include_directories(calibrationTest PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(calibrationTest LINK_PUBLIC ${OPENCV_LIBS})
