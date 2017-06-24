#include "Map_Optimization/g2omapoptimizer.h"

visual_slam::G2OMapOptimizer::G2OMapOptimizer()
{

  graph = std::unique_ptr<g2o::SparseOptimizer>(new g2o::SparseOptimizer);

  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

  graph->setAlgorithm(solver);

  // add the parameter representing the sensor offset
//  SE3 sensorOffsetTransf(0, 0, 0);
//  ParameterSE3Offset* sensorOffset = new ParameterSE3Offset;
//  sensorOffset->setOffset(sensorOffsetTransf);
//  sensorOffset->setId(0);
//  graph.addParameter(sensorOffset);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  VertexPose * v = new VertexPose();
  int poseNum2Id = uniqueId.getUniqueId();
  v->setEstimate(pose);
  v->setId(poseNum2Id);
  v->setFixed(true);
  graph->addVertex(v);
}

void visual_slam::G2OMapOptimizer::addPoseToGraph(Eigen::Isometry3d& pose, int& poseNum2Id)
{
  VertexPose * v = new VertexPose();
  poseNum2Id = uniqueId.getUniqueId();
  v->setEstimate(pose);
  v->setId(poseNum2Id);
  graph->addVertex(v);
}

void visual_slam::G2OMapOptimizer::addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b)
{
  EdgePosePose * e = new EdgePosePose;
  g2o::OptimizableGraph::Vertex * pose_a_vertex = dynamic_cast<g2o::OptimizableGraph::Vertex*>(graph->vertices()[id_a]);
  g2o::OptimizableGraph::Vertex * pose_b_vertex = dynamic_cast<g2o::OptimizableGraph::Vertex*>(graph->vertices()[id_b]);

  e->vertices()[0] = pose_a_vertex;
  e->vertices()[1] = pose_b_vertex;
  Eigen::Matrix<double, 6, 6> Lambda;
  Lambda.setIdentity();
  e->setMeasurement(a_T_b);
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
