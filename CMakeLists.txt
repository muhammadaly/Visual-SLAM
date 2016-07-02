cmake_minimum_required(VERSION 2.8.3)
project(visual_slam)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp
        pcl_ros
        sensor_msgs
        tf)
set(CMAKE_CXX_FLAGS "")

find_package(catkin REQUIRED COMPONENTS
        ${${PROJECT_NAME}_CATKIN_DEPS}
#        angles
#        pcl_conversions
#        pcl_ros
#        roscpp
#        sensor_msgs
#        roslaunch
#        std_msgs
#        genmsg
        )
catkin_package()

include_directories(include
        ${catkin_INCLUDE_DIRS})

#find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)
#find_package( OpenCV REQUIRED COMPONENTS)
#find_package(Boost)

set(OpenCV_INCLUDE_DIR "/usr/local/include")
include_directories(${OpenCV_INCLUDE_DIR})
set(OpenCV_LIBS
        "opencv_core;opencv_highgui;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_flann")
link_directories(/usr/local/lib)

#set(PCL_INCLUDE_DIR "/usr/include/pcl-1.7")
#include_directories(${PCL_INCLUDE_DIR})
#set(PCL_LIBS
#        "pcl_common;pcl_features")
#link_directories(/usr/lib)

#set(VTK_INCLUDE_DIR "/usr/include/vtk-5.8")
#include_directories(${VTK_INCLUDE_DIR})
#set(VTK_LIBS)
#link_directories(/usr/lib)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------------------------------------------------------
#add_executable (alignment_prerejective
#        src/alignment_prerejective.cpp
#        src/framedata.cpp
#        include/visual_slam/framedata.h )
#target_link_libraries(alignment_prerejective ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#        ${PCL_LIBS})
#-----------------------------------------------------------------------------------------------
add_executable (Feature_Ext_Mat
        src/Feature_Ext_Mat.cpp
        src/framedata.cpp
        include/visual_slam/framedata.h )
target_link_libraries(Feature_Ext_Mat ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

