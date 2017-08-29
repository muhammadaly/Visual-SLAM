#include "GraphManager.h"

visual_slam::GraphManager::GraphManager()
{
  reset_request_ = false;
  min_matches = 3;
  next_seq_id = 0;
  next_vertex_id = 0;
  graph_.clear();
}
bool visual_slam::GraphManager::addFrame(Frame* new_frame)
{
  if(reset_request_) resetGraph();

  if (new_frame->getKeypoints().size() < min_matches) {
      return false;
  }

  //First Node, so only build its index, insert into storage and add a
  //vertex at the origin, of which the position is very certain
  if (graph_.size()==0){
      firstNode(new_frame);
      return true;
  }

  //All other nodes are processed in the following
  QMatrix4x4 motion_estimate;///Output:contains the best-yet of the pairwise motion estimates for the current node
  bool edge_to_last_keyframe_found = false;
  bool found_match = nodeComparisons(new_frame, motion_estimate, edge_to_last_keyframe_found);

  if (found_match)
  { //Success
    if(localization_only_)
    {
      ROS_INFO("Localizing (only)");
      localizationUpdate(new_frame, motion_estimate);
    }
    else //Mapping
    {
      //This needs to be done before rendering, so deleting the cloud always works
      graph_[new_frame->id_] = new_frame; //Node->id_ == Graph_ Index
      //First render the cloud with the best frame-to-frame estimate
      //The transform will get updated when optimizeGraph finishes
      pointcloud_type* cloud_to_visualize = new_frame->pc_col.get();
      std_vector_of_eigen_vector4f * features_to_visualize = &(new_frame->feature_locations_3d_);
      if(!new_frame->valid_tf_estimate_) {
        cloud_to_visualize = new pointcloud_type();
        features_to_visualize = new std_vector_of_eigen_vector4f();
      }
      ROS_INFO("Adding node with id %i and seq id %i to the graph", new_frame->id_, new_frame->seq_id_);

      //Juergen: bad hack, should instead prevent the creation of the cloud, but this is faster implementation wise
      ROS_INFO_STREAM("create cloud " << new_frame->id_ << " " << ps->get<int>("create_cloud_every_nth_node") << " " << new_frame->id_%ps->get<int>("create_cloud_every_nth_node")) ;
      if((new_frame->id_%ps->get<int>("create_cloud_every_nth_node"))!=0){
        new_frame->clearPointCloud();
      }

      if(!edge_to_last_keyframe_found && earliest_loop_closure_node_ > keyframe_ids_.back()) {
        this->addKeyframe(new_frame->id_-1);//use the id of the node before, because that one is still localized w.r.t. a keyframe. So keyframes are connected
      } else {
        if(ps->get<bool>("visualize_keyframes_only")){
          cloud_to_visualize = new pointcloud_type();
          features_to_visualize = new std_vector_of_eigen_vector4f();
        }
      }
      if(ps->get<bool>("glwidget_without_clouds")){
        cloud_to_visualize = new pointcloud_type();
        features_to_visualize = new std_vector_of_eigen_vector4f();
      }

      Q_EMIT setPointCloud(cloud_to_visualize, motion_estimate);
      Q_EMIT setFeatures(features_to_visualize);
      ROS_INFO("Added Node, new graphsize: %i nodes", (int) graph_.size());
      if(ps->get<int>("optimizer_skip_step") > 0 &&
          (camera_vertices.size() % ps->get<int>("optimizer_skip_step")) == 0)
      {
        optimizeGraph();
      }

      //This is old stuff for visualization via rviz - not tested in a long time, would be safe to delete _if_ nobody uses it
      visualizeGraphEdges();
      visualizeGraphNodes();
      visualizeFeatureFlow3D(marker_id_++);

    }
  }
  else //Unsuccesful
  {
    if(graph_.size() == 1){//if there is only one node which has less features, replace it by the new one
      ROS_WARN("Choosing new initial node, because it has more features");
      if(new_frame->feature_locations_2d_.size() > graph_[0]->feature_locations_2d_.size()){
        this->resetGraph();
        process_node_runs_ = false;
        firstNode(new_frame);
        return true;
      }
    } else { //delete new_frame; //is now  done by auto_ptr
      ROS_WARN("Did not add as Node");
    }
  }

 return found_match;
}

void GraphManager::firstNode(FrameData* new_frame)
{
    new_frame->setId(graph_.size());
    new_frame->setSequenceId(next_seq_id++); // allways incremented, even if node is not added
    //init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
    //printTransform("Ground Truth Transform for First Node", init_base_pose_);

    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
    new_frame->vertex_id_ = next_vertex_id++;
    graph_[new_frame->getId()] = new_frame;
    reference_pose->setId(new_node->getVertexId());

    camera_vertices.insert(reference_pose);

    ROS_INFO("Adding initial node with id %i and seq %i, v_id: %i", new_node->id_, new_node->seq_id_, new_node->vertex_id_);
    g2o::SE3Quat g2o_ref_se3 = tf2G2O(init_base_pose_);
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex_.lock();
    optimizer_->addVertex(reference_pose);
    optimizer_mutex_.unlock();
    QString message;
    Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
    //pointcloud_type::Ptr the_pc(new_node->pc_col); //this would delete the cloud after the_pc gets out of scope
    QMatrix4x4 latest_transform = g2o2QMatrix(g2o_ref_se3);
    if(!ParameterServer::instance()->get<bool>("glwidget_without_clouds")) {
      Q_EMIT setPointCloud(new_node->pc_col.get(), latest_transform);
      Q_EMIT setFeatures(&(new_node->feature_locations_3d_));
    }
    //current_poses_.append(latest_transform);
    this->addKeyframe(new_node->id_);
    if(ParameterServer::instance()->get<bool>("octomap_online_creation")) {
      optimizeGraph(); //will do the following at the end:
      //updateCloudOrigin(new_node);
      //renderToOctomap(new_node);
    }

    process_node_runs_ = false;
}


