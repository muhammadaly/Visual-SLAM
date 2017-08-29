cmake_minimum_required(VERSION 2.8.3)
project(visual_slam)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    pcl_ros
    sensor_msgs
    pcl_conversions
    pcl_ros
    std_msgs
    eigen_conversions
    tf)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS filters io registration segmentation visualization)

set(OpenCV_INCLUDE_DIR "/usr/local/include")
set(OpenCV_LIBS  "opencv_core;opencv_highgui;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_flann;opencv_xfeatures2d")
set(VTK_INCLUDE_DIR "/usr/include/vtk-5.8")

include_directories(include/visual_slam
    ${VTK_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PNG_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${Cholmod_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    /usr/include/suitesparse
    )
LIST(APPEND G2O_LIBS
  cxsparse
  cholmod
  g2o_cli g2o_ext_freeglut_minimal g2o_simulator
  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
  g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
  g2o_types_sba g2o_types_slam3d g2o_csparse_extension
  g2o_opengl_helper g2o_solver_dense g2o_stuff
  g2o_types_sclam2d g2o_parser g2o_solver_pcg
  g2o_types_data g2o_types_sim3
)

catkin_package(CATKIN_DEPENDS roscpp
    pcl_ros
    sensor_msgs
    pcl_conversions
    eigen_conversions
    pcl_ros
    std_msgs
    tf)

link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/framedata.cpp
    src/robot_pose.cpp
    src/OpenNIListener.cpp
    src/Feature_Extraction/CVORBFeatureExtractorAndDescriptor.cpp
    src/Feature_Extraction/CVSURFFeatureExtractorAndDescriptor.cpp
    src/Feature_Extraction/CVSIFTFeatureExtractorAndDescriptor.cpp
    src/Feature_Extraction/pclfeaturedescriptor.cpp
    src/Feature_Extraction/pclfeatureextraction.cpp
    src/Transformation_Estimation/TransformationEstimator.cpp
    src/Transformation_Estimation/PCL3DRANSACTransformationEstimator.cpp
    src/Feature_Matching/cvflannfeaturematcher.cpp
    src/Feature_Matching/FeatureMatcher.cpp
    src/Utilities/PCLUtilities.cpp
    src/Utilities/LogFile.cpp
    src/custom_types/vertex_pose.cpp
    src/custom_types/edge_pose_pose.cpp
    src/Utilities/TUMUtilities.cpp
    src/Utilities/EigenUtilites.cpp
    src/Map_Optimization/g2omapoptimizer.cpp)

set(HEADERS include/visual_slam/framedata.h
    include/visual_slam/robot_pose.h
    include/visual_slam/OpenNIListener.h
    include/visual_slam/Feature_Extraction/FeatureExtractorAndDescriptor.h
    include/visual_slam/Feature_Extraction/CVORBFeatureExtractorAndDescriptor.h
    include/visual_slam/Feature_Extraction/CVSURFFeatureExtractorAndDescriptor.h
    include/visual_slam/Feature_Extraction/CVSIFTFeatureExtractorAndDescriptor.h
    include/visual_slam/Feature_Extraction/pclfeaturedescriptor.h
    include/visual_slam/Feature_Extraction/pclfeatureextraction.h
    include/visual_slam/Transformation_Estimation/TransformationEstimator.h
    include/visual_slam/Transformation_Estimation/PCL3DRANSACTransformationEstimator.h
    include/visual_slam/Feature_Matching/cvflannfeaturematcher.h
    include/visual_slam/Feature_Matching/FeatureMatcher.h
    include/visual_slam/Utilities/PCLUtilities.h
    include/visual_slam/Utilities/LogFile.h
    include/visual_slam/Utilities/TUMUtilities.h
    include/visual_slam/Utilities/EigenUtilites.h
    include/visual_slam/custom_types/vertex_pose.h
    include/visual_slam/custom_types/edge_pose_pose.h
    include/visual_slam/Map_Optimization/g2omapoptimizer.h)



#------------------------------------------------------
#add_executable(Transformation_Estimation_RANSAC src/Transformation_Estimation_RANSAC.cpp ${SOURCES} ${HEADERS})
#target_link_libraries(Transformation_Estimation_RANSAC ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------
#add_executable(Transformation src/Transformation.cpp ${SOURCES} ${HEADERS})
#target_link_libraries(Transformation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------
#add_executable(Feature_Ext_Mat src/Feature_Ext_Mat.cpp ${SOURCES} ${HEADERS})
#target_link_libraries(Feature_Ext_Mat ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#------------------------------------------------------
#add_executable(Transformation_Estimation src/Trajectory_Estimation.cpp ${SOURCES} ${HEADERS})
#target_link_libraries(Transformation_Estimation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${G2O_LIBS})
##------------------------------------------------------
#add_executable(Mapping src/Mapping.cpp ${SOURCES} ${HEADERS})
#target_link_libraries(Mapping ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${G2O_LIBS})
###------------------------------------------------------
add_executable(Trajectory_Estimation src/Trajectory_Estimation.cpp ${SOURCES} ${HEADERS})
target_link_libraries(Trajectory_Estimation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${G2O_LIBS})
#------------------------------------------------------
add_executable(transformation_tests src/transformation_tests.cpp ${SOURCES} ${HEADERS})
target_link_libraries(transformation_tests ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${G2O_LIBS})
#------------------------------------------------------
