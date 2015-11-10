cmake_minimum_required (VERSION 2.8)
project(StereoTests)

# Collect all sources
file(GLOB_RECURSE TRAININGAPP_SOURCE_FILES "src/*.cpp" "src/*.h" "src/*.inl" "../objectsMap/src/vision/*.h" "../objectsMap/src/vision/*.cpp" "../objectsMap/src/utils/*.h" "../objectsMap/src/utils/*.cpp")

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

# Loading 3rd party libraries.
find_package(OpenCV 3 REQUIRED core imgproc ml highgui features2d xfeatures2d)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
LINK_DIRECTORIES(${OpenCV_LIB_DIR})


find_package(PCL 1.2)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND})
	add_definitions(-DENABLE_PCL)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
endif()


# Add Executable
add_executable(trainingApp ${TRAININGAPP_SOURCE_FILES})

target_include_directories(trainingApp PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_include_directories(trainingApp PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(trainingApp PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/cjson/src)
target_link_libraries(trainingApp LINK_PUBLIC cjson)
target_link_libraries(trainingApp LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(trainingApp LINK_PUBLIC ${PCL_LIBRARIES})

