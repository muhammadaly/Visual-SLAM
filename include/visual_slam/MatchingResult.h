#ifndef MATCHINGRESULT_H
#define MATCHINGRESULT_H

#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include "edge.h"

namespace visual_slam {

class MatchingResult {
    public:
        MatchingResult() :
          rmse(0.0),
          ransac_trafo(Eigen::Matrix4f::Identity()),
          final_trafo(Eigen::Matrix4f::Identity()),
          icp_trafo(Eigen::Matrix4f::Identity()),
          inlier_points(0), outlier_points(0), occluded_points(0)
        {
            edge.id1 = edge.id2 = -1;
        }
        std::vector<cv::DMatch> inlier_matches;
        std::vector<cv::DMatch> all_matches;
        LoadedEdge3D edge;
        float rmse;
        Eigen::Matrix4f ransac_trafo;
        Eigen::Matrix4f final_trafo;
        Eigen::Matrix4f icp_trafo;
        unsigned int inlier_points, outlier_points, occluded_points, all_points;
        const char* toString();
};

}

#endif // MATCHINGRESULT_H
