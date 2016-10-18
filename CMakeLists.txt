cmake_minimum_required(VERSION 2.8.3)
project(visual_slam)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    pcl_ros
    sensor_msgs
    tf)
set(CMAKE_CXX_FLAGS "")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
  angles
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  roslaunch
  std_msgs
  genmsg
  laser_geometry
  dynamixel_msgs
)

find_package( PNG REQUIRED )
find_package(Boost)
find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS filters io registration segmentation visualization)
#find_package( OpenCV REQUIRED )

find_package( CUDA REQUIRED )

set(OpenCV_INCLUDE_DIR "/usr/local/include")
include_directories(${OpenCV_INCLUDE_DIR})
set(OpenCV_LIBS  "opencv_core;opencv_highgui;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_flann")
link_directories(/usr/local/lib)

set(CUDA_NVCC_FLAGS "-arch=sm_20" CACHE STRING "nvcc flags" FORCE)
set (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
#set(LIB_TYPE STATIC)
#cuda_add_library(TestLib ${PCL_LIBRARIES} ${LIB_TYPE} cuda/hello.cu)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)


include_directories(include
                /usr/include/vtk-5.8
                ${PCL_INCLUDE_DIRS}
                ${PNG_INCLUDE_DIRS}
                ${catkin_INCLUDE_DIRS}
                ${Boost_INCLUDE_DIR}
                ${CUDA_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#------------------------------------------------------
add_executable(Transformation_Estimation_RANSAC src/Transformation_Estimation_RANSAC.cpp
        src/framedata.cpp
        include/visual_slam/framedata.h)
target_link_libraries(Transformation_Estimation_RANSAC ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------
add_executable(Transformation_Estimation_Node src/Transformation_Estimation_Node.cpp)
target_link_libraries(Transformation_Estimation_Node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#------------------------------------------------------
add_executable(Feature_Ext_Mat src/Feature_Ext_Mat.cpp
        src/framedata.cpp
        include/visual_slam/framedata.h )
target_link_libraries(Feature_Ext_Mat ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------

