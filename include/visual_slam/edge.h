#ifndef EDGE_H
#define EDGE_H


#include <set>
#include <iostream>
#include <Eigen/Geometry>

struct LoadedEdge3D
{
  int id1, id2;
  //enum { OTHER=0, RANSAC, ICP } edge_type;
  //edge_type type;
  //g2o::SE3Quat mean;
  Eigen::Isometry3d transform;
  Eigen::Matrix<double, 6,6> informationMatrix;
};

struct LoadedEdgeComparator3D
{
  inline bool operator()(const LoadedEdge3D& e1, const LoadedEdge3D& e2){
    int i11=e1.id1, i12=e1.id2;
    if (i11>i12){
      i11=e1.id2;
      i12=e1.id1;
    }
    int i21=e2.id1, i22=e2.id2;
    if (i21>i22){
      i21=e2.id2;
      i22=e2.id1;
    }
    if (i12<i22) return true;
    if (i12>i22) return false;
    return (i11<i21);
  }
};

typedef std::set<LoadedEdge3D, LoadedEdgeComparator3D> LoadedEdgeSet3D;


#endif // EDGE_H
