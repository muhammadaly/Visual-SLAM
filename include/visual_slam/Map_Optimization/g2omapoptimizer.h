#ifndef G2OMAPOPTIMIZER_H
#define G2OMAPOPTIMIZER_H

#include <Eigen/Core>
#include <definitions.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "custom_types/edge_pose_pose.h"

namespace visual_slam {
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;

class G2OMapOptimizer
{
public:
  G2OMapOptimizer();
  void addPoseToGraph(Eigen::Isometry3d& pose, int& poseNum2Id);
  void addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b);
  void optimize();
  std::vector<visual_slam::Pose_6D> getPoses();

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
}
#endif // G2OMAPOPTIMIZER_H
