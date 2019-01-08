cmake_minimum_required(VERSION 3.12)
project(PAIGE)

#include_directories("/usr/local/opt/libomp/include")
#link_directories("/usr/local/opt/libomp/lib")

find_package(Shark REQUIRED)
include(${SHARK_USE_FILE})

include_directories(./)
include_directories("${PROJECT_SOURCE_DIR}/libomp7.0.0/include")
link_directories("${PROJECT_SOURCE_DIR}/libomp7.0.0/lib")

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

find_package(OpenMVG REQUIRED)

add_executable(PAIGE main.cpp PAIGE_cal.h PAIGE_cal.cpp Definition.h PAIGE_Feature.cpp PAIGE_Feature.h PAIGE_Processor.cpp PAIGE_Processor.h PAIGE_test.cpp PAIGE_test.h Histogram_Block.h)

target_link_libraries( PAIGE
        PRIVATE
        ${SHARK_LIBRARIES}
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_image
        )

#set_property(TARGET PAIGE PROPERTY CXX_STANDARD 11)
#target_link_libraries(PAIGE ${SHARK_LIBRARIES})

