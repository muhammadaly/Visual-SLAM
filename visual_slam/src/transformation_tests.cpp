#include <iostream>

#include "ros/ros.h"
#include "robot_pose.h"
#include "definitions.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//ros::Publisher odom_pub ;

//void publishOdometry(visual_slam::TFMatrix robot_pose)
//{
//  double x = static_cast<double>(robot_pose(0,3));
//  double y = static_cast<double>(robot_pose(1,3));
//  double z = static_cast<double>(robot_pose(2,3));

//  double qw = sqrt(1.0 + robot_pose(0,0) + robot_pose(1,1) + robot_pose(2,2)) / 2.0;
//  double w4 = 4.0 * qw ;
//  double qx = (robot_pose(2,1) - robot_pose(1,2)) / w4;
//  double qy = (robot_pose(0,2) - robot_pose(2,0)) / w4;
//  double qz = (robot_pose(1,0) - robot_pose(0,1)) / w4;

//  nav_msgs::Odometry odom;


//  odom.header.stamp = ros::Time::now();
//  odom.header.frame_id = "odom";
//  odom.child_frame_id = "base_link";

//  //set the position
//  odom.pose.pose.position.x = x;
//  odom.pose.pose.position.y = y;
//  odom.pose.pose.position.z = z;
//  odom.pose.pose.orientation.w = w4;
//  odom.pose.pose.orientation.x = qx;
//  odom.pose.pose.orientation.y = qy;
//  odom.pose.pose.orientation.z = qz;

//  //publish the message
//  odom_pub.publish(odom);
//}

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "transformation_tests");
//  visual_slam::Robot_Pose pose;
//  ros::NodeHandle _node;
//  double tX = 0.0;
//  double tY = 0.0;
//  double tZ = 0.0;
//  double rRoll = pose.convertFromDegreeToRadian(0.0);
//  double rPitch = pose.convertFromDegreeToRadian(0.0);
//  double rYaw = pose.convertFromDegreeToRadian(90.0);
//  visual_slam::TFMatrix tf = pose.getTransformationMatrix(tX,tY,tZ,rRoll,rPitch,rYaw);
//  odom_pub = _node.advertise<nav_msgs::Odometry>("odom", 50);

//  while(ros::ok())
//  {
//    pose *= tf;
//    publishOdometry(pose.getTransformationMatrix());
//  }
////  std::cout << pose.getTransformationMatrix() << std::endl;

//  ros::spin();
//  return 0;
//}

int main(int argc, char** argv){
  ros::init(argc, argv, "transformation_tests");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);

  visual_slam::Robot_Pose pose(0.0,0.0,0.5,0.0,0.0,0.0);
  double rYaw = pose.convertFromDegreeToRadian(60);
  visual_slam::TFMatrix tf = pose.getTransformationMatrix(0.0,0.0,0.0,0.0,0.0,rYaw);

  while(n.ok()){

    ros::spinOnce();
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    visual_slam::TFMatrix Inverse_transformation = pose.getInverseTransformation(tf);

    pose *= Inverse_transformation;

    visual_slam::RobotPose6D robotPose = pose.getRobotPose();
    x = robotPose(0);
    y = robotPose(1);
    th = robotPose(5);
    ROS_INFO("%f " , th);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = robotPose(0);
    odom_trans.transform.translation.y = robotPose(1);
    odom_trans.transform.translation.z = robotPose(2);
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = robotPose(0);
    odom.pose.pose.position.y = robotPose(1);
    odom.pose.pose.position.z = robotPose(2);
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    //odom.twist.twist.linear.x = vx;
    //odom.twist.twist.linear.y = vy;
    //odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
