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

    int sequentially_previous_id = graph_.rbegin()->second->getGraphNodeId();
    MatchingResult mr;
    int prev_best = mr.edge.id1;
    curr_best_result_ = mr;

    //Initial Comparison ######################################################################
    bool predecessor_matched = false;
    if(min_translation_meter > 0.0 || min_rotation_degree > 0.0)
    {
      //First check if trafo to last frame is big
      FrameData* prev_frame = graph_[graph_.size()-1];
      if(localization_only_ && curr_best_result_.edge.id1 > 0){ prev_frame =  graph_[curr_best_result_.edge.id1]; }
      ROS_INFO("Comparing new node (%i) with previous node %i", new_node->id_, prev_frame->id_);
      mr = new_node->matchNodePair(prev_frame);
      ROS_INFO("Node comparison result: %s", mr.toString());
      if(mr.edge.id1 >= 0 && mr.edge.id2 >= 0) {//Found trafo
        ros::Time time1 = prev_frame->header_.stamp;
        ros::Time time2 = new_node->header_.stamp;
        ros::Duration delta_time =  time2 - time1;
        if(!EigenUtilities->instance()->isBigTranfo(mr.edge.transform) ||
           !EigenUtilities->instance()->isSmallTrafo(mr.edge.transform, delta_time.toSec())){ //Found trafo, but bad trafo (too small to big)
            ROS_WARN("Transformation not within bounds. Did not add as Node");
            tf::Transform incremental = EigenUtilities->instance()->eigenTransf2TF(mr.edge.transform);
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[prev_frame->id_]->vertex_id_));
            tf::Transform previous = EigenUtilities->instance()->eigenTransf2TF(v->estimate());
            tf::Transform combined = previous*incremental;
            latest_transform_cache_ = stampedTransformInWorldFrame(new_node, combined);
            //printTransform("Computed new transform", latest_transform_cache_);
            broadcastTransform(latest_transform_cache_);
            process_node_runs_ = false;
            curr_best_result_ = mr;
            return false;
        }
        else
        { //Good Transformation
          ROS_DEBUG_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << ") \n" << mr.edge.informationMatrix);
          if (addEdgeToG2O(mr.edge, prev_frame, new_node,  true, true, curr_motion_estimate)) 
          {
            graph_[new_node->id_] = new_node; //Needs to be added
            if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
//#ifdef DO_FEATURE_OPTIMIZATION
//            updateLandmarks(mr, prev_frame,new_node);
//#endif
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



tf::StampedTransform visual_slam::GraphManager::stampedTransformInWorldFrame(const Node *node, const tf::Transform &computed_motion) const
{
  std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
  std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
  if(base_frame.empty()){ //if there is no base frame defined, use frame of sensor data
    base_frame = node->header_.frame_id;
  }
  const tf::StampedTransform& base2points = node->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time

  tf::Transform world2base = init_base_pose_*base2points*computed_motion*base2points.inverse();
  //printTransform("World->Base", world2base);

  return tf::StampedTransform(world2base.inverse(), base2points.stamp_, base_frame, fixed_frame);
}

void visual_slam::GraphManager::broadcastTransform(const tf::StampedTransform &computed_motion)
{
  br_.sendTransform(stamped_transform);
  if(graph_.size() > 0){
    Node* current_node = graph_.at(graph_.size() - 1);
    if(current_node && current_node->header_.stamp.toSec() == stamped_transform.stamp_.toSec()){
      publishCloud(current_node, current_node->header_.stamp, online_cloud_pub_);
    } else {
      ROS_WARN("Timestamp of transform does not match node");
    }
  }
}
bool visual_slam::GraphManager::addEdgeToG2O(const LoadedEdge3D& edge,
                                Node* n1, Node* n2,
                                bool largeEdge, bool set_estimate,
                                QMatrix4x4& motion_estimate) //Pure output
{
    ScopedTimer s(__FUNCTION__);
    assert(n1);
    assert(n2);
    assert(n1->id_ == edge.id1);
    assert(n2->id_ == edge.id2);

    QMutexLocker locker(&optimizer_mutex_);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n1->vertex_id_));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n2->vertex_id_));

    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("Edge to new vertex is too short, vertex will not be inserted");
            return false;
        }
    }

    if(!v1 && !v2){
      ROS_ERROR("Missing both vertices: %i, %i, cannot create edge", edge.id1, edge.id2);
      return false;
    }
    else if (!v1 && v2) {
        v1 = new g2o::VertexSE3;
        assert(v1);
        int v_id = next_vertex_id++;
        v1->setId(v_id);

        n1->vertex_id_ = v_id; // save vertex id in node so that it can find its vertex
        v1->setEstimate(v2->estimate() * edge.transform.inverse());
        camera_vertices.insert(v1);
        optimizer_->addVertex(v1);
        motion_estimate = eigenTF2QMatrix(v1->estimate());
        ROS_WARN("Creating previous id. This is unexpected by the programmer");
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        int v_id = next_vertex_id++;
        v2->setId(v_id);
        n2->vertex_id_ = v_id;
        v2->setEstimate(v1->estimate() * edge.transform);
        camera_vertices.insert(v2);
        optimizer_->addVertex(v2);
        motion_estimate = eigenTF2QMatrix(v2->estimate());
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimate() * edge.transform);
        motion_estimate = eigenTF2QMatrix(v2->estimate());
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    Eigen::Isometry3d meancopy(edge.transform);
    g2o_edge->setMeasurement(meancopy);
    //Change setting from which mahal distance the robust kernel is used: robust_kernel_.setDelta(1.0);
    g2o_edge->setRobustKernel(&robust_kernel_);
    // g2o_edge->setInverseMeasurement(edge.trannsform.inverse());
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);
    //ROS_DEBUG_STREAM("Added Edge ("<< edge.id1 << "-" << edge.id2 << ") to Optimizer:\n" << edge.transform << "\nInformation Matrix:\n" << edge.informationMatrix);
    cam_cam_edges_.insert(g2o_edge);
    current_match_edges_.insert(g2o_edge); //Used if all previous vertices are fixed ("pose_relative_to" == "all")
//    new_edges_.append(qMakePair(edge.id1, edge.id2));

    if(abs(edge.id1 - edge.id2) > ParameterServer::instance()->get<int>("predecessor_candidates")){
      loop_closures_edges++;
    } else {
      sequential_edges++;
    }
    current_edges_.append( qMakePair(n1->id_, n2->id_));

    if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "inaffected") {
      v1->setFixed(false);
      v2->setFixed(false);
    }
    else if(ParameterServer::instance()->get<std::string>("pose_relative_to") == "largest_loop") {
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id1);
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id2);
    }
    return true;
}

void visual_slam::GraphManager::updateInlierFeatures(const MatchingResult& mr, Node* new_node, Node* old_node)
{
  BOOST_FOREACH(const cv::DMatch& match, mr.inlier_matches){
    assert(new_node->feature_matching_stats_.size() > match.queryIdx);
    assert(old_node->feature_matching_stats_.size() > match.trainIdx);
    unsigned char& new_flag = new_node->feature_matching_stats_[match.queryIdx];
    if(new_flag < 255) ++new_flag;
    unsigned char& old_flag = old_node->feature_matching_stats_[match.trainIdx];
    if(old_flag < 255) ++old_flag;
  }
}

QList<int> GraphManager::getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int sequential_targets, int geodesic_targets, int sampled_targets, int predecessor_id, bool include_predecessor)
{
    QList<int> ids_to_link_to; //return value
    if(predecessor_id < 0) predecessor_id = graph_.size()-1;
    //Prepare output
    std::stringstream ss;
    ss << "Node ID's to compare with candidate for node " << graph_.size() << ". Sequential: ";

   if((int)camera_vertices.size() <= sequential_targets+geodesic_targets+sampled_targets ||
      camera_vertices.size() <= 1)
    { //if less prev nodes available than targets requestet, just use all
      sequential_targets = sequential_targets+geodesic_targets+sampled_targets;
      geodesic_targets = 0;
      sampled_targets = 0;
      predecessor_id = graph_.size()-1;
    }

    if(sequential_targets > 0){
      //all the sequential targets (will be checked last)
      for (int i=1; i < sequential_targets+1 && predecessor_id-i >= 0; i++) {
          ids_to_link_to.push_back(predecessor_id-i);
          ss << ids_to_link_to.back() << ", " ;
      }
    }

    if(geodesic_targets > 0){
      g2o::HyperDijkstra hypdij(optimizer_);
      g2o::UniformCostFunction cost_function;
      g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[predecessor_id]->vertex_id_));
      hypdij.shortestPaths(prev_vertex,&cost_function,ParameterServer::instance()->get<int>("geodesic_depth"));
      g2o::HyperGraph::VertexSet& vs = hypdij.visited();

      //Need to convert vertex_id to node id
      std::map<int, int> vertex_id_to_node_id;
      for (graph_it it = graph_.begin(); it !=graph_.end(); ++it){
            Node *node = it->second;
            vertex_id_to_node_id[node->vertex_id_] = node->id_;
            //ROS_WARN("ID Pair: %d, %d", node->vertex_id_, node->id_);
      }

      //Geodesic Neighbours except sequential
      std::map<int,int> neighbour_indices; //maps neighbour ids to their weights in sampling
      int sum_of_weights=0;
      for (g2o::HyperGraph::VertexSet::iterator vit=vs.begin(); vit!=vs.end(); vit++) { //FIXME: Mix of vertex id and graph node (with features) id
        int vid = (*vit)->id();
        //ROS_WARN("Vertex ID: %d", vid);
        int id = 0;
        try{
          id = vertex_id_to_node_id.at(vid);
        }
        catch (std::exception e){//Catch exceptions: Unexpected problems shouldn't crash the application
          ROS_ERROR("Vertex ID %d has no corresponding node", vid);
          ROS_ERROR("Map Content:");
          for(std::map<int,int>::const_iterator it = vertex_id_to_node_id.begin(); it != vertex_id_to_node_id.end(); it++){
            ROS_ERROR("Node ID %d: Vertex ID %d", it->first, it->second);
          }
          for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); it++)
          {
            ROS_ERROR("Vertex ID %d", it->first);
          }
        }
        if(!graph_.at(id)->matchable_) continue;
        if(id < predecessor_id-sequential_targets || (id > predecessor_id && id <= (int)graph_.size()-1)){ //Geodesic Neighbours except sequential
            int weight = abs(predecessor_id-id);
            neighbour_indices[id] = weight; //higher probability to be drawn if far away
            sum_of_weights += weight;
        }
      }

      //Sample targets from graph-neighbours
      ss << "Dijkstra: ";
      while(ids_to_link_to.size() < sequential_targets+geodesic_targets && neighbour_indices.size() != 0){
        int random_pick = rand() % sum_of_weights;
        ROS_DEBUG("Pick: %d/%d", random_pick, sum_of_weights);
        int weight_so_far = 0;
        for(std::map<int,int>::iterator map_it = neighbour_indices.begin(); map_it != neighbour_indices.end(); map_it++ ){
          weight_so_far += map_it->second;
          ROS_DEBUG("Checking: %d, %d, %d", map_it->first, map_it-> second, weight_so_far);
          if(weight_so_far > random_pick){//found the selected one
            int sampled_id = map_it->first;
            ids_to_link_to.push_front(sampled_id);
            ss << ids_to_link_to.front() << ", " ;
            sum_of_weights -= map_it->second;
            ROS_DEBUG("Taking ID: %d, decreasing sum of weights to %d", map_it->first, sum_of_weights);
            neighbour_indices.erase(map_it);
            ROS_ERROR_COND(sum_of_weights<0, "Sum of weights should never be zero");
            break;
          }
          ROS_DEBUG("Skipping ID: %d", map_it->first);
        }//for
      }
    }

    if(sampled_targets > 0){
      ss << "Random Sampling: ";
      std::vector<int> non_neighbour_indices;//initially holds all, then neighbours are deleted
      non_neighbour_indices.reserve(graph_.size());
      for (QList<int>::iterator it = keyframe_ids_.begin(); it != keyframe_ids_.end(); it++){
        if(ids_to_link_to.contains(*it) == 0 && graph_.at(*it)->matchable_){
          non_neighbour_indices.push_back(*it);
        }
      }

      //Sample targets from non-neighbours (search new loops)
      while(ids_to_link_to.size() < geodesic_targets+sampled_targets+sequential_targets && non_neighbour_indices.size() != 0){
          int index_of_v_id = rand() % non_neighbour_indices.size();
          int sampled_id = non_neighbour_indices[index_of_v_id];
          non_neighbour_indices[index_of_v_id] = non_neighbour_indices.back(); //copy last id to position of the used id
          non_neighbour_indices.resize(non_neighbour_indices.size()-1); //drop last id
          ids_to_link_to.push_front(sampled_id);
          ss << ids_to_link_to.front() << ", " ;
      }
    }

    if(include_predecessor){
      ids_to_link_to.push_back(predecessor_id);
      ss << predecessor_id;
    }
    ROS_INFO("%s", ss.str().c_str());
    return ids_to_link_to; //only compare to first frame
}

