# eigen3
FIND_PACKAGE(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.8 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

## Specify additional locations of header files
include_directories(
    ${PCL_INCLUDE_DIRS}
)


## add libraries
add_library(utils
    utils.cpp
)
target_link_libraries(utils
)

## add libraries
add_library(calcFeatures
    calc_features.cpp
)
target_link_libraries(calcFeatures
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

## add executables
add_executable(pc_viewer
    pc_viewer.cpp
)
target_link_libraries(pc_viewer
    ${PCL_LIBRARIES}
    utils
)

add_executable(pc_segment
    pc_segment.cpp
)
target_link_libraries(pc_segment
    ${PCL_LIBRARIES}
    utils
)


add_executable(generate_features
    generate_features.cpp
)
target_link_libraries(generate_features
    ${PCL_LIBRARIES}
    calcFeatures
    utils
)


add_executable(train_rtrees
    train_rtrees.cpp
)
target_link_libraries(train_rtrees
    ${OpenCV_LIBS}
    utils
)
