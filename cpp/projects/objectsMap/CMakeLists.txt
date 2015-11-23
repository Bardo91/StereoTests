cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(StereoTests)

# Collect all sources
file(GLOB_RECURSE OBJECTSMAP_SOURCE_FILES "src/*.cpp" "src/*.h" "src/*.inl")

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
find_package(OpenCV 3 REQUIRED core imgproc ml highgui features2d xfeatures2d calib3d objdetect)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

find_package(PCL 1.2 REQUIRED common io kdtree filters visualization segmentation)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND})
	add_definitions(-DENABLE_PCL)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
endif()

# Copy configuration file
file(COPY AppConfig.json DESTINATION ${CMAKE_BINARY_DIR}/projects/objectsMap)

# Add Executable
add_executable(objectsMap ${OBJECTSMAP_SOURCE_FILES})

target_include_directories(objectsMap PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_include_directories(objectsMap PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(objectsMap PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/cjson/src)
target_include_directories(objectsMap PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/libsvmpp/src)
target_link_libraries(objectsMap LINK_PUBLIC cjson)
target_link_libraries(objectsMap LINK_PUBLIC libsvmpp)
target_link_libraries(objectsMap LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(objectsMap LINK_PUBLIC ${PCL_LIBRARIES})