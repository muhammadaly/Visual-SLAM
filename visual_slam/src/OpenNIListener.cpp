#include "OpenNIListener.h"

#include "Utilities/OpenCVUtilities.h"

void visual_slam::OpenNIListener::setupSubscribers(){
  int queue;
  _node.getParam("/queue_size", queue);
  std::string bagfile_name;
  _node.getParam("/bagfile_name", bagfile_name);
  if(bagfile_name.empty()){

    std::string visua_tpc, depth_tpc, cinfo_tpc, cloud_tpc;
    _node.getParam("topic_image_mono", visua_tpc);
    _node.getParam("topic_image_depth", depth_tpc);
    _node.getParam("camera_info_topic", cinfo_tpc);
    _node.getParam("topic_points", cloud_tpc);

    //All information from Kinect
    if(!visua_tpc.empty() && !depth_tpc.empty() && !cloud_tpc.empty())
    {
      visua_sub_ = new image_sub_type(_node, visua_tpc, q);
      depth_sub_ = new image_sub_type (_node, depth_tpc, q);
      cloud_sub_ = new pc_sub_type (_node, cloud_tpc, q);
      kinect_sync_ = new message_filters::Synchronizer<KinectSyncPolicy>(KinectSyncPolicy(q),  *visua_sub_, *depth_sub_, *cloud_sub_),
          kinect_sync_->registerCallback(boost::bind(&OpenNIListener::kinectCallback, this, _1, _2, _3));
      ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to " << visua_tpc << ", " << depth_tpc << " and " << cloud_tpc);
    }
    //No cloud, but visual image and depth
    else if(!visua_tpc.empty() && !depth_tpc.empty() && !cinfo_tpc.empty() && cloud_tpc.empty())
    {
      visua_sub_ = new image_sub_type(_node, visua_tpc, q);
      depth_sub_ = new image_sub_type(_node, depth_tpc, q);
      cinfo_sub_ = new cinfo_sub_type(_node, cinfo_tpc, q);
      no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *visua_sub_, *depth_sub_, *cinfo_sub_);
      no_cloud_sync_->registerCallback(boost::bind(&OpenNIListener::noCloudCallback, this, _1, _2, _3));
      ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to " << visua_tpc << " and " << depth_tpc);
    }
  }
  else {
    ROS_WARN("RGBDSLAM loads a bagfile - therefore doesn't subscribe to topics.");
  }
}

void OpenNIListener::kinectCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,
                                     const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  if(visual_img.rows != depth_float_img.rows ||
     visual_img.cols != depth_float_img.cols ||
     point_cloud->width != (uint32_t) visual_img.cols ||
     point_cloud->height != (uint32_t) visual_img.rows){
    ROS_ERROR("PointCloud, depth and visual image differ in size! Ignoring Data");
    return;
  }
  image_encoding_ = visual_img_msg->encoding;
  OpenCVUtilities::instance()->depthToCV8UC1(depth_float_img, depth_mono8_img_);

  if(ROSUtilities::instance()->asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp))
    return;

  PointCloudT::Ptr pc_col(new PointCloudT());//will belong to node
  pcl::fromROSMsg(*point_cloud,*pc_col);
  cameraCallback(visual_img, pc_col, depth_mono8_img_);
}

void OpenNIListener::cameraCallback(cv::Mat visual_img,
                                    PointCloudT::Ptr point_cloud,
                                    cv::Mat depth_mono8_img)
{
  FrameData* frame = new FrameData(visual_img, depth_mono8_img, point_cloud);

  retrieveTransformations(pcl_conversions::fromPCL(point_cloud->header), frame);
  processFrame(visual_img, frame);
}

void OpenNIListener::retrieveTransformations(std_msgs::Header depth_header, FrameData* frame)
{
  std::string base_frame, odom_frame, gt_frame;
  _node.getParam("base_frame_name", base_frame);
  _node.getParam("odom_frame_name", odom_frame);
  _node.getParam("ground_truth_frame_name",gt_frame);

  std::string depth_frame_id = depth_header.frame_id;
  ros::Time depth_time = depth_header.stamp;
  tf::StampedTransform base2points;

  try{
    tflistener_->waitForTransform(base_frame, depth_frame_id, depth_time, ros::Duration(0.005));
    tflistener_->lookupTransform(base_frame, depth_frame_id, depth_time, base2points);
    base2points.stamp_ = depth_time;
  }
  catch (tf::TransformException ex){
    base2points.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    base2points.setOrigin(tf::Point(0,-0.04,0));
    base2points.stamp_ = depth_time;
    base2points.frame_id_ = base_frame;
    base2points.child_frame_id_ = depth_frame_id;
  }
  frame->setBase2PointsTransform(base2points);

  if(!gt_frame.empty()){
    //Retrieve the ground truth data. For the first frame it will be
    //set as origin. the rest will be used to compare
    tf::StampedTransform ground_truth_transform;
    try{
      tflistener_->waitForTransform(gt_frame, "/openni_camera", depth_time, ros::Duration(0.005));
      tflistener_->lookupTransform(gt_frame, "/openni_camera", depth_time, ground_truth_transform);
      ground_truth_transform.stamp_ = depth_time;
      tf::StampedTransform b2p;
      //HACK to comply with Jürgen Sturm's Ground truth, though I can't manage here to get the full transform from tf
      b2p.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
      b2p.setOrigin(tf::Point(0,-0.04,0));
      ground_truth_transform *= b2p;
    }
    catch (tf::TransformException ex){
      ROS_WARN_THROTTLE(5, "%s - Using Identity for Ground Truth (This message is throttled to 1 per 5 seconds)",ex.what());
      ground_truth_transform = tf::StampedTransform(tf::Transform::getIdentity(), depth_time, "missing_ground_truth", "/openni_camera");
    }
    printTransform("Ground Truth", ground_truth_transform);
    frame->setGroundTruthTransform(ground_truth_transform);
  }
  if(!odom_frame.empty()){
    //Retrieve the ground truth data. For the first frame it will be
    //set as origin. the rest will be used to compare
    tf::StampedTransform current_odom_transform;
    try{
      tflistener_->waitForTransform(depth_frame_id, odom_frame, depth_time, ros::Duration(0.005));
      tflistener_->lookupTransform( depth_frame_id, odom_frame, depth_time, current_odom_transform);
    }
    catch (tf::TransformException ex){
      ROS_WARN_THROTTLE(5, "%s - No odometry available (This message is throttled to 1 per 5 seconds)",ex.what());
      current_odom_transform = tf::StampedTransform(tf::Transform::getIdentity(), depth_time, "missing_odometry", depth_frame_id);
      current_odom_transform.stamp_ = depth_time;
    }
    printTransform("Odometry", current_odom_transform);
    frame->setOdomTransform(current_odom_transform);
  }
}


void OpenNIListener::processFrame(FrameData* new_node)
{
  bool has_been_added = graph_mgr_->addNode(new_node);
  ++num_processed_;
  std::string odom_frame_name;
  _node.getParam("odom_frame_name", odom_frame_name);
  if(has_been_added && !odom_frame_name.empty()){
    ROS_INFO_NAMED("OpenNIListener", "Verifying Odometry Information");
    ros::Time latest_odom_time;
    std::string odom_frame  = ps->get<std::string>("odom_frame_name");
    std::string base_frame  = ps->get<std::string>("base_frame_name");
    std::string error_msg;
    int ev = tflistener_->getLatestCommonTime(odom_frame, base_frame, latest_odom_time, &error_msg);
    if(ev == tf::NO_ERROR){
      graph_mgr_->addOdometry(latest_odom_time, tflistener_);
    } else {
      ROS_WARN_STREAM("Couldn't get time of latest tf transform between " << odom_frame << " and " << base_frame << ": " << error_msg);
    }
  }
}
