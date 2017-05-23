#include "visual_slam/Map_Optimization/g2omapoptimizer.h"

G2OMapOptimizer::G2OMapOptimizer()
{
  graph = std::unique_ptr<g2o::SparseOptimizer>(new g2o::SparseOptimizer);
}

void G2OMapOptimizer::addPoseToGraph(Eigen::Isometry3d& pose, int& poseNum2Id)
{
  VertexPose * v = new VertexPose();

  // get id
  poseNum2Id = uniqueId.getUniqueId();

  // populate g2o vertex object and add to graph
  v->setEstimate(pose);
  v->setId(poseNum2Id);
  v->setFixed(false);
  graph->addVertex(v);
}

void G2OMapOptimizer::addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b)
{
  EdgePosePose * e = new EdgePosePose;

  // retrieve vertex pointers from graph with id's
  g2o::OptimizableGraph::Vertex * pose_a_vertex
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (graph->vertices()[id_a]);

  g2o::OptimizableGraph::Vertex * pose_b_vertex
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (graph->vertices()[id_b]);

  // error check vertices and associate them with the edge
  assert(pose_a_vertex!=NULL);
  assert(pose_a_vertex->dimension() == 6);
  e->vertices()[0] = pose_a_vertex;

  assert(pose_b_vertex!=NULL);
  assert(pose_b_vertex->dimension() == 6);
  e->vertices()[1] = pose_b_vertex;

  // add information matrix
  Eigen::Matrix<double, 6, 6> Lambda;
  Lambda.setIdentity();

  // set the observation and imformation matrix
  e->setMeasurement(a_T_b);
  e->information() = Lambda;

  // finally add the edge to the graph
  if(!graph->addEdge(e))
  {
    assert(false);
  }
}

void G2OMapOptimizer::optimize()
{
  graph->initializeOptimization();
  graph->setVerbose(true);
  graph->optimize(10);
}

std::vector<Pose_6D> G2OMapOptimizer::getPoses()
{
  std::vector<Pose_6D> poses ;
  for(int i = 0 ; i < graph->vertices().size() ; i++)
  {
    VertexPose * pose
        = dynamic_cast<VertexPose*>
        (graph->vertices()[i]);
    poses.push_back(pose->estimate());

  }
  return poses;
}
