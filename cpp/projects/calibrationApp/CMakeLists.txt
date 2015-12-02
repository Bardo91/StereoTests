cmake_minimum_required (VERSION 2.8)
project(StereoTests)

# Collect all sources
file(GLOB_RECURSE CALIBRATIONAPP_SOURCE_FILES "src/*.cpp" "src/*.h" "src/*.inl" "../objectsMap/src/vision/*.h" "../objectsMap/src/vision/*.cpp" "../objectsMap/src/utils/*.h" "../objectsMap/src/utils/*.cpp")

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

# Loading 3rd party libraries.
find_package(OpenCV 3 REQUIRED core imgproc ml highgui features2d xfeatures2d calib3d objdetect)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
LINK_DIRECTORIES(${OpenCV_LIB_DIR})


# Add Executable
add_executable(calibrationApp ${CALIBRATIONAPP_SOURCE_FILES})

target_include_directories(calibrationApp PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_include_directories(calibrationApp PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(calibrationApp LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(calibrationApp LINK_PUBLIC ${PCL_LIBRARIES})

