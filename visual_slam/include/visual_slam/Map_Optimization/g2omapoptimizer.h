#ifndef G2OMAPOPTIMIZER_H
#define G2OMAPOPTIMIZER_H

#include <Eigen/Core>
#include <visual_slam/definitions.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <visual_slam/custom_types/vertex_pose.h>
#include <visual_slam/custom_types/edge_pose_pose.h>

class G2OMapOptimizer
{
public:
  G2OMapOptimizer();
  void addPoseToGraph(Eigen::Isometry3d& pose, int& poseNum2Id);
  void addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b);
  void optimize();
  std::vector<Pose_6D> getPoses();

private :
  std::unique_ptr<g2o::SparseOptimizer> graph;
};

class UniqueId
{
public:
  UniqueId():unique_id(0){}
  int getUniqueId()
  {
    return unique_id++;
  }
private:
  int unique_id;
};
static UniqueId uniqueId;

#endif // G2OMAPOPTIMIZER_H
