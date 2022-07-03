cmake_minimum_required(VERSION 2.8.3)
project(rrxio)

find_package(catkin_simple REQUIRED)
find_package(OpenCV)

catkin_simple()

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -march=native")
set(ROVIO_NMAXFEATURE 15 CACHE STRING "Number of features for ROVIO")
set(ROVIO_NCAM 1 CACHE STRING "Number of enabled cameras")
set(ROVIO_NLEVELS 4 CACHE STRING "Number of image leavels for the features")
set(ROVIO_PATCHSIZE 6 CACHE STRING "Size of patch (edge length in pixel)")
set(ROVIO_NPOSE 0 CACHE STRING "Additional estimated poses for external pose measurements")
add_definitions(-DROVIO_NMAXFEATURE=${ROVIO_NMAXFEATURE})
add_definitions(-DROVIO_NCAM=${ROVIO_NCAM})
add_definitions(-DROVIO_NLEVELS=${ROVIO_NLEVELS})
add_definitions(-DROVIO_PATCHSIZE=${ROVIO_PATCHSIZE})
add_definitions(-DROVIO_NPOSE=${ROVIO_NPOSE})

cs_add_executable(rrxio_rosbag_loader_10 src/nodes/rrxio_rosbag_loader.cpp)
target_compile_definitions(rrxio_rosbag_loader_10 PRIVATE ROVIO_NMAXFEATURE=10 ROVIO_UPDATE_SOURCE=0)
target_link_libraries(rrxio_rosbag_loader_10 ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenCV_LIBRARIES})

cs_add_executable(rrxio_rosbag_loader_15 src/nodes/rrxio_rosbag_loader.cpp)
target_compile_definitions(rrxio_rosbag_loader_15 PRIVATE ROVIO_NMAXFEATURE=15 ROVIO_UPDATE_SOURCE=0)
target_link_libraries(rrxio_rosbag_loader_15 ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenCV_LIBRARIES})

cs_add_executable(rrxio_rosbag_loader_25 src/nodes/rrxio_rosbag_loader.cpp)
target_compile_definitions(rrxio_rosbag_loader_25 PRIVATE ROVIO_NMAXFEATURE=25 ROVIO_UPDATE_SOURCE=0)
target_link_libraries(rrxio_rosbag_loader_25 ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenCV_LIBRARIES})

## umcomment the following lines for online mode

#cd_add_executable(rrxio_node_10 src/nodes/rrxio_node.cpp)
#target_compile_definitions(rrxio_node_10 PRIVATE ROVIO_NMAXFEATURE=10 ROVIO_UPDATE_SOURCE=0)
#target_link_libraries(rrxio_node_10 ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenCV_LIBRARIES})

#cd_add_executable(rrxio_node_15 src/nodes/rrxio_node.cpp)
#target_compile_definitions(rrxio_node_15 PRIVATE ROVIO_NMAXFEATURE=15 ROVIO_UPDATE_SOURCE=0)
#target_link_libraries(rrxio_node_15 ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenCV_LIBRARIES})

#cd_add_executable(rrxio_node_25 src/nodes/rrxio_node.cpp)
#target_compile_definitions(rrxio_node_25 PRIVATE ROVIO_NMAXFEATURE=25 ROVIO_UPDATE_SOURCE=0)
#target_link_libraries(rrxio_node_25 ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenCV_LIBRARIES})

cs_install()

cs_export(INCLUDE_DIRS include)

## qtcreator project tree
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

file(GLOB_RECURSE CFG "cfg/*")
add_custom_target(CFG SOURCES ${CFG})

file(GLOB_RECURSE LAUNCH "launch/*")
add_custom_target(LAUNCH SOURCES ${LAUNCH})

file(GLOB_RECURSE PYTHON "python/*")
add_custom_target(PYTHON SOURCES ${PYTHON})

file(GLOB_RECURSE CONFIG "config/*")
add_custom_target(CONFIG SOURCES ${CONFIG})
