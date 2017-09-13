#ifndef GRAPHMANAGER_H
#define GRAPHMANAGER_H

#include "framedata.h"
#include <tf/Transform.h>
#include <Utilities/EigenUtilites.h>

namespace visual_slam {

class GraphManager
{
public:
  GraphManager();
  bool addFrame(FrameData* new_frame);

  tf::StampedTransform stampedTransformInWorldFrame(const Node* node,
                                                    const tf::Transform& computed_motion) const;
  void broadcastTransform(const tf::StampedTransform& computed_motion);

private:
  int min_matches;
  bool reset_request_;
  unsigned int next_seq_id;
  unsigned int next_vertex_id;
  int earliest_loop_closure_node_;
  tf::Transform  init_base_pose_;
  unsigned int marker_id_;
  bool localization_only_;

  bool valid_tf_estimate_;

  visual_slam::MatchingResult curr_best_result_;

  std::vector<int> keyframe_ids_;
  bool process_node_runs_;

  std::map<int, FrameData* > graph_;
  g2o::HyperGraph::VertexSet camera_vertices;
  g2o::HyperGraph::EdgeSet cam_cam_edges_;


  QMutex optimizer_mutex_;
};

}
#endif // GRAPHMANAGER_H
