cmake_minimum_required(VERSION 3.12)
project(PAIGE)

#include_directories("/usr/local/opt/libomp/include")
#link_directories("/usr/local/opt/libomp/lib")

find_package(OpenMVG REQUIRED)
find_package(Shark REQUIRED)

include(${SHARK_USE_FILE})
#message(${SHARK_USE_FILE})

#include_directories(./)


#include_directories("${PROJECT_SOURCE_DIR}/libomp7.0.0/include")
#link_directories("${PROJECT_SOURCE_DIR}/libomp7.0.0/lib")

set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


add_executable(PAIGE_and_GroundTruth PAIGE_and_GroundTruth.cpp PAIGE_Feature.cpp PAIGE_Feature.h PAIGE_Processor.cpp PAIGE_Processor.h PAIGE_test.cpp PAIGE_test.h Histogram_Block.h Histogram_Block_io.h )

target_link_libraries( PAIGE_and_GroundTruth
        PRIVATE
        ${SHARK_LIBRARIES}
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_system
        )


add_executable(PAIGE_Pair_From_Images PAIGE_Pair_From_Images.cpp PAIGE_Feature.cpp PAIGE_Feature.h PAIGE_Processor.cpp PAIGE_Processor.h PAIGE_test.cpp PAIGE_test.h Histogram_Block.h Histogram_Block_io.h)

target_link_libraries( PAIGE_Pair_From_Images
        PRIVATE
        ${SHARK_LIBRARIES}
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_system
        )

add_executable(PAIGE_pair_From_SfM_data PAIGE_pair_From_SfM_data.cpp  PAIGE_Feature.cpp PAIGE_Feature.h PAIGE_Processor.cpp PAIGE_Processor.h PAIGE_test.cpp PAIGE_test.h Histogram_Block.h Histogram_Block_io.h)

target_link_libraries(PAIGE_pair_From_SfM_data
        PRIVATE
        ${SHARK_LIBRARIES}
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_system
        )


#set_property(TARGET PAIGE PROPERTY CXX_STANDARD 11)
#target_link_libraries(PAIGE ${SHARK_LIBRARIES})







