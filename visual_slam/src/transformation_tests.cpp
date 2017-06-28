#include <iostream>

#include "ros/ros.h"
#include "robot_pose.h"
#include "definitions.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformation_tests");
//  visual_slam::Robot_Pose pose(0,0,0,0,0.785398,0);

  visual_slam::Robot_Pose pose;
  visual_slam::RobotRotationAngles3D angles(0,0,1.5708);
  visual_slam::RobotTranslation3D translation(0,0,0);
  pose.setRotationAngles(angles);
  pose.setTranslationVector(translation);
  visual_slam::TFMatrix tf = pose.getTransformationMatrix();
  pose = visual_slam::Robot_Pose(0,0,0,0,0,0);

  std::cout<<pose.getRobotPose()<<std::endl;
  pose *= tf;
  std::cout<<"After Transformation -----"<<std::endl;
  std::cout<<pose.getRobotPose()<<std::endl;

  ros::spin();
  return 0;
}
