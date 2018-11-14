CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -march=native -std=c++11")

FIND_PACKAGE(OpenCV REQUIRED)

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/build")

include_directories(./include
        )

ADD_EXECUTABLE(Video_Mosaic_LK
        src/Affine.cpp
        src/gaussj.cpp
        src/LaplacianPyramid.cpp
        src/main.cpp
        src/reg.cpp
        src/ROI.cpp
        src/Stablizer.cpp
        src/StablizerWrapper.cpp
        )

target_link_libraries(Video_Mosaic_LK
        ${OpenCV_LIBS}
        )