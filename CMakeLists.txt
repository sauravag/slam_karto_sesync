cmake_minimum_required(VERSION 2.8.3)
project(slam_karto_sesync)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    visualization_msgs
)

add_subdirectory(${PROJECT_SOURCE_DIR}/SESync)

find_package(Eigen3 3.3 REQUIRED)

include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_filters nav_msgs open_karto rosconsole roscpp sensor_msgs geometry_msgs tf visualization_msgs 
 DEPENDS Eigen3
)
add_executable(slam_karto_sesync src/slam_karto_sesync.cpp src/SESyncSolver.cpp src/SESyncOptimizer.cpp)

target_link_libraries(slam_karto_sesync SESync roptlib ${BLAS_LIBRARIES} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES} ${LAPACK} ${catkin_LIBRARIES})

#install(TARGETS slam_karto_sesync
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)