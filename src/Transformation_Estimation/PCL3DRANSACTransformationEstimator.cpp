#include <Transformation_Estimation/PCL3DRANSACTransformationEstimator.h>

bool visual_slam::PCL3DRANSACTransformationEstimator::estimateTransformation(visual_slam::PointCloudT::Ptr firstPC, visual_slam::FeatureCloudT::Ptr firstPCFeatures, visual_slam::PointCloudT::Ptr secondPC,
                                                                visual_slam::FeatureCloudT::Ptr secondPCFeatures, visual_slam::TFMatrix & transformation,visual_slam::PointCloudT::Ptr& alignedPC)
{

  const float leaf = 0.005f;
  int MaximumIterations = 50000; // Number of RANSAC iterations
  int NumberOfSamples = 3; // Number of points to sample for generating/prerejecting a pose
  int CorrespondenceRandomness = 5; // Number of nearest features to use
  float SimilarityThreshold = 0.9f; // Polygonal edge length similarity threshold
  float MaxCorrespondenceDistance = 2.5f* leaf; // Inlier threshold 0.015
  float InlierFraction = 0.25f; // Required inlier fraction for accepting a pose hypothesis

  pcl::SampleConsensusPrerejective<PointT,PointT,FeatureT> align;
  align.setInputSource(secondPC);
  align.setSourceFeatures(secondPCFeatures);
  align.setInputTarget(firstPC);
  align.setTargetFeatures(firstPCFeatures);

  align.setMaximumIterations(MaximumIterations); // Number of RANSAC iterations
  align.setNumberOfSamples(NumberOfSamples); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(CorrespondenceRandomness); // Number of nearest features to use
  align.setSimilarityThreshold(SimilarityThreshold); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(MaxCorrespondenceDistance); // Inlier threshold
  align.setInlierFraction(InlierFraction); // Required inlier fraction for accepting a pose hypothesis
  align.align(*alignedPC);

  if(align.hasConverged())
  {
    transformation = align.getFinalTransformation();
    return true;
  }
  else
  {
//    pcl::console::print_error("Alignment failed!\n");
    return false;
  }
}
