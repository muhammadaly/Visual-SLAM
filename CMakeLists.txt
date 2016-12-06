cmake_minimum_required(VERSION 2.8.3)
project(visual_slam)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    pcl_ros
    sensor_msgs
    pcl_conversions
    pcl_ros
    std_msgs)
set(CMAKE_CXX_FLAGS "")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS filters io registration segmentation visualization)

set(OpenCV_INCLUDE_DIR "/usr/local/include")
set(OpenCV_LIBS  "opencv_core;opencv_highgui;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_flann")

include_directories(include
                /usr/include/vtk-5.8
                ${OpenCV_INCLUDE_DIR}
                ${PCL_INCLUDE_DIRS}
                ${PNG_INCLUDE_DIRS}
                ${catkin_INCLUDE_DIRS}
                ${Boost_INCLUDE_DIR})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)
add_definitions(${PCL_DEFINITIONS})


#------------------------------------------------------
add_executable(Transformation_Estimation_RANSAC src/Transformation_Estimation_RANSAC.cpp
        src/framedata.cpp
        include/visual_slam/framedata.h)
target_link_libraries(Transformation_Estimation_RANSAC ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------
add_executable(Transformation src/Transformation.cpp
        src/framedata.cpp
        include/visual_slam/framedata.h)
target_link_libraries(Transformation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------
add_executable(Transformation_Estimation_Node src/Transformation_Estimation_Node.cpp)
target_link_libraries(Transformation_Estimation_Node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#------------------------------------------------------
add_executable(Feature_Ext_Mat src/Feature_Ext_Mat.cpp
        src/framedata.cpp
        include/visual_slam/framedata.h )
target_link_libraries(Feature_Ext_Mat ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------

