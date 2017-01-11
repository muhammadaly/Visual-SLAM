#include <visual_slam/Transformation_Estimation/PCL3DRANSACTransformationEstimator.h>

void PCL3DRANSACTransformationEstimator::estimateTransformation(PointCloudT::Ptr firstPC, FeatureCloudT::Ptr firstPCFeatures, PointCloudT::Ptr secondPC, FeatureCloudT::Ptr secondPCFeatures, TFMatrix & transformation)
{
  PointCloudT::Ptr object_aligned (new PointCloudT);
  pcl::SampleConsensusPrerejective<PointT,PointT,FeatureT> align;
  align.setInputSource (secondPC);
  align.setSourceFeatures (secondPCFeatures);
  align.setInputTarget (firstPC);
  align.setTargetFeatures (firstPCFeatures);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  align.align (*object_aligned);
  if (align.hasConverged ())
    transformation = align.getFinalTransformation ();
  else
    pcl::console::print_error ("Alignment failed!\n");
}
