#include "Map_Optimization/g2omapoptimizer.h"

visual_slam::G2OMapOptimizer::G2OMapOptimizer()
{
  graph = std::unique_ptr<g2o::SparseOptimizer>(new g2o::SparseOptimizer);
  graph->setVerbose(false);

  visual_slam::SlamLinearCholmodSolver* linearSolver = new visual_slam::SlamLinearCholmodSolver();
  linearSolver->setBlockOrdering(false);
  visual_slam::SlamBlockSolver* solver = new SlamBlockSolver(linearSolver);

  g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(solver);
  graph->setAlgorithm(algo);

  g2o::ParameterCamera* cameraParams = new g2o::ParameterCamera();
  cameraParams->setKcam(fx, fy, cx, cy);
  g2o::SE3Quat offset; // identity
  cameraParams->setOffset(offset);
  cameraParams->setId(0);
  graph->addParameter(cameraParams);
}

void visual_slam::G2OMapOptimizer::addPoseToGraph(Eigen::Isometry3d& pose, int& poseNum2Id)
{
  g2o::VertexSE3 * v = new g2o::VertexSE3;
  poseNum2Id = uniqueId.getUniqueId();
  v->setEstimate(pose);
  v->setId(poseNum2Id);
  v->setFixed(false);
  graph->addVertex(v);
}

void visual_slam::G2OMapOptimizer::addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b)
{
  g2o::EdgeSE3 * e = new g2o::EdgeSE3;
  g2o::VertexSE3 * pose_a_vertex = dynamic_cast<g2o::VertexSE3*>(graph->vertices()[id_a]);
  g2o::VertexSE3 * pose_b_vertex = dynamic_cast<g2o::VertexSE3*>(graph->vertices()[id_b]);

  e->vertices()[0] = pose_a_vertex;
  e->vertices()[1] = pose_b_vertex;
  e->setMeasurement(a_T_b);

  Eigen::Matrix<double, 6, 6> Lambda;
  Lambda.setIdentity();
  e->setInformation(Lambda);

  graph->addEdge(e);
}

void visual_slam::G2OMapOptimizer::optimize()
{
  graph->initializeOptimization();
  graph->setVerbose(true);
  graph->optimize(10);
}

std::vector<visual_slam::Pose_6D> visual_slam::G2OMapOptimizer::getPoses()
{
  std::vector<visual_slam::Pose_6D> poses ;
  for(int i = 0 ; i < graph->vertices().size() ; i++)
  {
    VertexPose * pose
        = dynamic_cast<VertexPose*>
        (graph->vertices()[i]);
    poses.push_back(pose->estimate());

  }
  return poses;
}
