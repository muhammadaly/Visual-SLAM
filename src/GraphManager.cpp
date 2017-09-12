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
  //if(reset_request_) resetGraph();

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

//  if (found_match)
//  { //Success
//    if(localization_only_)
//    {
//      ROS_INFO("Localizing (only)");
//      localizationUpdate(new_frame, motion_estimate);
//    }
//    else //Mapping
//    {
//      //This needs to be done before rendering, so deleting the cloud always works
//      graph_[new_frame->id_] = new_frame; //Node->id_ == Graph_ Index
//      //First render the cloud with the best frame-to-frame estimate
//      //The transform will get updated when optimizeGraph finishes
//      pointcloud_type* cloud_to_visualize = new_frame->pc_col.get();
//      std_vector_of_eigen_vector4f * features_to_visualize = &(new_frame->feature_locations_3d_);
//      if(!new_frame->valid_tf_estimate_) {
//        cloud_to_visualize = new pointcloud_type();
//        features_to_visualize = new std_vector_of_eigen_vector4f();
//      }
//      ROS_INFO("Adding node with id %i and seq id %i to the graph", new_frame->id_, new_frame->seq_id_);

//      //Juergen: bad hack, should instead prevent the creation of the cloud, but this is faster implementation wise
//      ROS_INFO_STREAM("create cloud " << new_frame->id_ << " " << ps->get<int>("create_cloud_every_nth_node") << " " << new_frame->id_%ps->get<int>("create_cloud_every_nth_node")) ;
//      if((new_frame->id_%ps->get<int>("create_cloud_every_nth_node"))!=0){
//        new_frame->clearPointCloud();
//      }

//      if(!edge_to_last_keyframe_found && earliest_loop_closure_node_ > keyframe_ids_.back()) {
//        this->addKeyframe(new_frame->id_-1);//use the id of the node before, because that one is still localized w.r.t. a keyframe. So keyframes are connected
//      } else {
//        if(ps->get<bool>("visualize_keyframes_only")){
//          cloud_to_visualize = new pointcloud_type();
//          features_to_visualize = new std_vector_of_eigen_vector4f();
//        }
//      }
//      if(ps->get<bool>("glwidget_without_clouds")){
//        cloud_to_visualize = new pointcloud_type();
//        features_to_visualize = new std_vector_of_eigen_vector4f();
//      }

//      Q_EMIT setPointCloud(cloud_to_visualize, motion_estimate);
//      Q_EMIT setFeatures(features_to_visualize);
//      ROS_INFO("Added Node, new graphsize: %i nodes", (int) graph_.size());
//      if(ps->get<int>("optimizer_skip_step") > 0 &&
//          (camera_vertices.size() % ps->get<int>("optimizer_skip_step")) == 0)
//      {
//        optimizeGraph();
//      }

//      //This is old stuff for visualization via rviz - not tested in a long time, would be safe to delete _if_ nobody uses it
//      visualizeGraphEdges();
//      visualizeGraphNodes();
//      visualizeFeatureFlow3D(marker_id_++);

//    }
//  }
//  else //Unsuccesful
//  {
//    if(graph_.size() == 1){//if there is only one node which has less features, replace it by the new one
//      ROS_WARN("Choosing new initial node, because it has more features");
//      if(new_frame->feature_locations_2d_.size() > graph_[0]->feature_locations_2d_.size()){
//        this->resetGraph();
//        process_node_runs_ = false;
//        firstNode(new_frame);
//        return true;
//      }
//    } else { //delete new_frame; //is now  done by auto_ptr
//      ROS_WARN("Did not add as Node");
//    }
//  }

// return found_match;
}

void GraphManager::firstNode(FrameData* new_frame)
{
    new_frame->setId(graph_.size());
    new_frame->setSequenceId(next_seq_id++); // allways incremented, even if node is not added
    init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
    //printTransform("Ground Truth Transform for First Node", init_base_pose_);

    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
    new_frame->vertex_id_ = next_vertex_id++;
    graph_[new_frame->getId()] = new_frame;
    reference_pose->setId(new_node->getVertexId());

    camera_vertices.insert(reference_pose);

    //ROS_INFO("Adding initial node with id %i and seq %i, v_id: %i", new_frame->getId(), new_frame->getSequenceId(), new_frame->getVertexId());
    g2o::SE3Quat g2o_ref_se3 = tf2G2O(init_base_pose_);
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex_.lock();
    optimizer_->addVertex(reference_pose);
    optimizer_mutex_.unlock();

    this->addKeyframe(new_node->id_);

    process_node_runs_ = false;
}

void GraphManager::addKeyframe(int id, bool pClearNonKeyframes)
{
  if(pClearNonKeyframes){
//    if(keyframe_ids_.size() >= 2){
//      int most_recent= keyframe_ids_.back();
//      int second_most_recent= keyframe_ids_.at(keyframe_ids_.size() - 2);
//      ROS_INFO("Clearing out data for nodes between keyframes %d and %d", second_most_recent, most_recent);
//      for (std::map<int, Node*>::iterator it=graph_.begin(); it!=graph_.end(); ++it){
//        Node* mynode = it->second;
//        if(mynode->id_ > second_most_recent && mynode->id_ < most_recent){
//          //mynode->getMemoryFootprint(true);//print
//          mynode->clearPointCloud();
//          mynode->clearFeatureInformation();
//        }
//      }
//    }
  }

  keyframe_ids_.push_back(id);
}

bool GraphManager::nodeComparisons(FrameData* new_node, 
                                   QMatrix4x4& curr_motion_estimate,
                                   bool& edge_to_keyframe)///Output:contains the best-yet of the pairwise motion estimates for the current node
{
    /// \callergraph
    process_node_runs_ = true;

    int num_keypoints = (int)new_node->feature_locations_2d_.size();
    if (num_keypoints < min_matches) && 
        ! keep_all_nodes))
    {
        ROS_INFO("Found only %i features on image, node is not included", num_keypoints);
        process_node_runs_ = false;
        return false;
    }

    //setting of the node id needs to be done here as the graph size can change inside this method
    new_node->setGraphNodeId(graph_.size());
    new_node->setSequence_id(next_seq_id++); // allways incremented, even if node is not added


    earliest_loop_closure_node_ = new_node->getGraphNodeId();
    unsigned int num_edges_before = cam_cam_edges_.size();
    edge_to_keyframe = false; //not yet found
    marker_id_ = 0; //overdraw old markers
    ROS_DEBUG("Graphsize: %d Nodes", (int) graph_.size());

    int sequentially_previous_id = graph_.rbegin()->second->id_; 
    MatchingResult mr;
    int prev_best = mr.edge.id1;
    curr_best_result_ = mr;

    //Initial Comparison ######################################################################
    bool predecessor_matched = false;
    if(min_translation_meter > 0.0 || min_rotation_degree > 0.0)
    {
      //First check if trafo to last frame is big
      //Node* prev_frame = graph_[graph_.size()-1];
      FrameData* prev_frame = graph_[graph_.size()-1];
      if(localization_only_ && curr_best_result_.edge.id1 > 0){ prev_frame =  graph_[curr_best_result_.edge.id1]; }
      ROS_INFO("Comparing new node (%i) with previous node %i", new_node->id_, prev_frame->id_);
      mr = new_node->matchNodePair(prev_frame);
      ROS_INFO("Node comparison result: %s", mr.toString());
      if(mr.edge.id1 >= 0 && mr.edge.id2 >= 0) {//Found trafo
        ros::Time time1 = prev_frame->header_.stamp;
        ros::Time time2 = new_node->header_.stamp;
        ros::Duration delta_time =  time2 - time1;
        if(!isBigTrafo(mr.edge.transform) || !isSmallTrafo(mr.edge.transform, delta_time.toSec())){ //Found trafo, but bad trafo (too small to big)
            ROS_WARN("Transformation not within bounds. Did not add as Node");
            //Send the current pose via tf nevertheless
            tf::Transform incremental = eigenTransf2TF(mr.edge.transform);
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[prev_frame->id_]->vertex_id_));
            tf::Transform previous = eigenTransf2TF(v->estimate());
            tf::Transform combined = previous*incremental;
            latest_transform_cache_ = stampedTransformInWorldFrame(new_node, combined);
            printTransform("Computed new transform", latest_transform_cache_);
            broadcastTransform(latest_transform_cache_);
            process_node_runs_ = false;
            curr_best_result_ = mr;
            return false;
        } else { //Good Transformation
          ROS_DEBUG_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << ") \n" << mr.edge.informationMatrix);
          if (addEdgeToG2O(mr.edge, prev_frame, new_node,  true, true, curr_motion_estimate)) 
          {
            graph_[new_node->id_] = new_node; //Needs to be added
            if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
#ifdef DO_FEATURE_OPTIMIZATION
            updateLandmarks(mr, prev_frame,new_node);
#endif
            updateInlierFeatures(mr, new_node, prev_frame);
            graph_[mr.edge.id1]->valid_tf_estimate_ = true;
            ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
            curr_best_result_ = mr;
            //addOutliers(Node* new_node, mr.inlier_matches);

          } else {
            ROS_INFO("Edge not added");
            process_node_runs_ = false;
            return false;
          }
        }
        predecessor_matched = true;
      }
      else {
        ROS_WARN("Found no transformation to predecessor (edge ids are negative)");
      // No transformation to predecessor. No other choice than try other candidates. This is done below 
      }
    }//end: Initial Comparison ######################################################################

    //Eigen::Matrix4f ransac_trafo, final_trafo;
    QList<int> vertices_to_comp;
    int  seq_cand = localization_only_ ? 0 : ps->get<int>("predecessor_candidates") - 1; //minus one, because the first predecessor has already been checked
    int geod_cand = ps->get<int>("neighbor_candidates");
    int samp_cand = ps->get<int>("min_sampled_candidates");
    if(predecessor_matched){
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 
    } else {
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, sequentially_previous_id, true); 
    }
    if(prev_best >= 0 && !vertices_to_comp.contains(prev_best)){
      vertices_to_comp.append(prev_best);//Test: definitely reuse best (currently: the oldest) matched node from last
    }

    QList<const Node* > nodes_to_comp;//only necessary for parallel computation

    //MAIN LOOP: Compare node pairs ######################################################################
    if (ps->get<bool>("concurrent_edge_construction")) 
    {
        std::stringstream ss;
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            //First compile a qlist of the nodes to be compared, then run the comparisons in parallel,
            //collecting a qlist of the results (using the blocking version of mapped).
            nodes_to_comp.push_front(graph_[vertices_to_comp[id_of_id]]); 
            ss << vertices_to_comp[id_of_id] << ", ";
        }
        ROS_INFO_STREAM("Nodes to compare: " << ss.str());
        QThreadPool* qtp = QThreadPool::globalInstance();
        ROS_INFO("Running node comparisons in parallel in %i (of %i) available threads", qtp->maxThreadCount() - qtp->activeThreadCount(), qtp->maxThreadCount());
        if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) {
            //Never seen this problem...
            ROS_WARN("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
            qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
        }
        QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&Node::matchNodePair, new_node, _1));

        for (int i = 0; i < results.size(); i++) 
        {
            MatchingResult& mr = results[i];
            ROS_INFO("Result of comparison %d: %s", i, mr.toString());
            if (mr.edge.id1 >= 0 ) {
              //ROS_INFO("new node has id %i", new_node->id_);
              assert(graph_[mr.edge.id1]);

              ros::Duration delta_time = new_node->header_.stamp - graph_[mr.edge.id1]->header_.stamp;
              if (isSmallTrafo(mr.edge.transform, delta_time.toSec()) &&
                  addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.transform), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
                { 
                  graph_[new_node->id_] = new_node; //Needs to be added
                  if(mr.edge.id1 == mr.edge.id2-1 ) {//older == newer-1
                    predecessor_matched = true;
                  }

#ifdef DO_FEATURE_OPTIMIZATION
                  updateLandmarks(mr, graph_[mr.edge.id1],new_node);
#endif
                  updateInlierFeatures(mr, new_node, graph_[mr.edge.id1]);
                  graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                  ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                  if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
                  //if (curr_best_result_.edge.id1 == -1 || sqrTransNorm(mr.final_trafo) < sqrTransNorm(curr_best_result_.final_trafo)) {
                    curr_best_result_ = mr;
                  }
                  if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
                }
                else{
                  ROS_WARN("Rejected edge from %d to %d", mr.edge.id1, mr.edge.id2);
                }
            }
        }
    } else { //Nonconcurrent
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            Node* node_to_compare = graph_[vertices_to_comp[id_of_id]];
            ROS_INFO("Comparing new node (%i) with node %i / %i", new_node->id_, vertices_to_comp[id_of_id], node_to_compare->id_);
            MatchingResult mr = new_node->matchNodePair(node_to_compare);
            ROS_INFO("Result of comparison: %s", mr.toString());

            if (mr.edge.id1 >= 0) {
              ros::Duration delta_time = new_node->header_.stamp - graph_[mr.edge.id1]->header_.stamp;
              if (isSmallTrafo(mr.edge.transform, delta_time.toSec()) &&
                  addEdgeToG2O(mr.edge, node_to_compare, new_node, isBigTrafo(mr.edge.transform), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
              {
#ifdef DO_FEATURE_OPTIMIZATION
                updateLandmarks(mr, node_to_compare, new_node);
#endif
                graph_[new_node->id_] = new_node; //Needs to be added
                if(mr.edge.id1 == mr.edge.id2-1 ) {//older == newer-1
                  predecessor_matched = true;
                }
                updateInlierFeatures(mr, new_node, node_to_compare);
                graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
                  //if (curr_best_result_.edge.id1 == -1 || sqrTransNorm(mr.final_trafo) < sqrTransNorm(curr_best_result_.final_trafo)) {
                  curr_best_result_ = mr;
                }
                if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
              }
              else {
                ROS_INFO("Matching result rejected for being too big? Time Delta: %f", delta_time.toSec());
              }
            }
            else 
            {
              ROS_INFO("Matching result rejected for edge.id1");
            }
        }
    }

    //END OF MAIN LOOP: Compare node pairs ######################################################################
    bool found_trafo = (cam_cam_edges_.size() != num_edges_before);
    bool valid_odometry = !ps->get<std::string>("odom_frame_name").empty();// || odom_tf_old.frame_id_ == "missing_odometry" || odom_tf_new.frame_id_ == "missing_odometry"; 

    bool keep_anyway = (ps->get<bool>("keep_all_nodes") || 
                        (((int)new_node->feature_locations_3d_.size() > ps->get<int>("min_matches")) 
                         && ps->get<bool>("keep_good_nodes")));
    ros::Duration delta_time = new_node->header_.stamp - graph_[sequentially_previous_id]->header_.stamp;
    float time_delta_sec = fabs(delta_time.toSec());
    ROS_WARN_COND(time_delta_sec >= 0.1, "Time jump (time delta: %.2f)", time_delta_sec);

    //If no trafo is found, only keep if a parameter says so or odometry is available. 
    //Otherwise only add a constant position edge, if the predecessor wasn't matched and its timestamp is nearby
    if((!found_trafo && valid_odometry) || 
       ((!found_trafo && keep_anyway) || 
        (!predecessor_matched && time_delta_sec < 0.1))) //FIXME: Add parameter for constant position assumption and time_delta
    { 
      LoadedEdge3D odom_edge;

      odom_edge.id1 = sequentially_previous_id;
      odom_edge.id2 = new_node->id_;
      odom_edge.transform.setIdentity();
      curr_motion_estimate = eigenTF2QMatrix(odom_edge.transform);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Zero(); 
      ROS_WARN("No valid (sequential) transformation between %d and %d: Using constant position assumption.", odom_edge.id1, odom_edge.id2);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity() / time_delta_sec;//e-9; 
      addEdgeToG2O(odom_edge,graph_[sequentially_previous_id],new_node, true,true, curr_motion_estimate);
      graph_[new_node->id_] = new_node; //Needs to be added
      new_node->valid_tf_estimate_ = false; //Don't use for postprocessing, rendering etc
      MatchingResult mr;
      mr.edge = odom_edge;
      curr_best_result_ = mr;
    }

    return cam_cam_edges_.size() > num_edges_before;
}


