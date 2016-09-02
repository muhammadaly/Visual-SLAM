#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <visual_slam/framedata.h>
#include <glob.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

static const int ORBFeatureVectorLength = 32;

typedef pcl::PointXYZ PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::Histogram<64> FeatureT;
//typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
//typedef pcl::visualization::PointCloudColorHandlerCustom<FeatureT> ColorHandlerT;

static const float fx = 525.0;  // focal length x
static const float fy = 525.0;  // focal length y
static const float cx = 319.5;  // optical center x
static const float cy = 239.5;  // optical center y

std::string rgbImages = "/home/ivsystems/master dataset/rgbd_dataset_freiburg1_xyz/rgb";
std::string depthImages = "/home/ivsystems/master dataset/rgbd_dataset_freiburg1_xyz/depth";
std::string transformationMatrices = "/home/ivsystems/master dataset/Results/UsingPCL/transformationMatrices.txt";
std::string featureMatching = "/home/ivsystems/master dataset/rgbd_dataset_freiburg1_xyz/matching";

std::ofstream myfile;

void showImage(cv::Mat img)
{
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", img );                   // Show our image inside it.

    cv::waitKey(0);
}
void showDescriptor(cv::Mat pDescriptors)
{
    int descriptorInd = 0;
    for(int Ind = 0 ; Ind < pDescriptors.cols ; Ind++)
    {
        printf("%f ," , pDescriptors.at<float>(descriptorInd,Ind));
    }
    printf("\n");
}
void computeFeatreORBFeatureDescriptor(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors , cv::Ptr<cv::ORB> orb)
{
    cv::Mat img;

    img = pFrameData.getFrameMatrix();
    orb->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
}

void computeFeatreSURFFeatureDescriptor(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors)
{
    //    cv::Mat img;

    //    img = pFrameData.getFrameMatrix();
    //    int minHessian = 400;

    //    cv::SurfFeatureDetector detector( minHessian );

    //    detector.detect( img, tkeypoint );

    //    SurfDescriptorExtractor extractor;
    //    extractor.compute( img, tkeypoint, tdescriptors );
}

void computeFeatreBRISKFeatureDescriptor(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors)
{
    cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
    cv::Mat img;

    img = pFrameData.getFrameMatrix();
    brisk->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);

}

void showMatchingImage(std::vector<cv::DMatch> good_matches
                       , std::vector<cv::KeyPoint> currentImageKeypoints , std::vector<cv::KeyPoint> previousImageKeypoints
                       , FrameData currentFrame, FrameData previousFrame)
{

    cv::Mat currentImage = currentFrame.getFrameMatrix();
    cv::Mat previousImage = previousFrame.getFrameMatrix();

    cv::namedWindow("matches", 1);
    cv::Mat img_matches;
    cv::drawMatches(currentImage, currentImageKeypoints, previousImage, previousImageKeypoints, good_matches, img_matches);
    cv::imshow("matches", img_matches);
    cv::waitKey(0);
}

void matchTwoCVDescriptors(cv::Mat currentImageDescriptors , cv::Mat previousImageDescriptors
                           , std::vector<cv::KeyPoint> currentImageKeypoints , std::vector<cv::KeyPoint> previousImageKeypoints
                           , FrameData currentFrame, FrameData previousFrame)
{
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;

    if (!currentImageDescriptors.empty() && !previousImageDescriptors.empty()) {
        if (currentImageDescriptors.type() != CV_32F) {
            currentImageDescriptors.convertTo(currentImageDescriptors, CV_32F);
        }
        if (previousImageDescriptors.type() != CV_32F) {
            previousImageDescriptors.convertTo(previousImageDescriptors, CV_32F);
        }
        matcher.match(currentImageDescriptors, previousImageDescriptors, matches);

        double max_dist = 0;
        double min_dist = 100;

        for (int i = 0; i < matches.size(); i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }

        std::vector<cv::DMatch> good_matches;

        for (int i = 0; i < matches.size(); i++) {
            if (matches[i].distance < 10 * min_dist) { good_matches.push_back(matches[i]); }
        }

        std::cout << matches.size() << std::endl;
    }
}
void matchTwoPCLDescriptors(FeatureCloudT::Ptr model_descriptors , FeatureCloudT::Ptr scene_descriptors)
{
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<FeatureT> match_search;
    match_search.setInputCloud (model_descriptors);
    //    int l = (int) scene_descriptors->size ();
    //    printf("%i," , l);
    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (int i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);

        if (!pcl_isfinite (scene_descriptors->at(i).histogram[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

}
FeatureCloudT::Ptr createFeaturePointCloud( cv::Mat pDescriptors)
{
    FeatureCloudT::Ptr FeaturePointCloud ( new FeatureCloudT);
    //    printf("rows : %i , cols : %i\n" , pDescriptors.rows , pDescriptors.cols);
    for(int KeyPointsInd = 0 ; KeyPointsInd < pDescriptors.rows ; KeyPointsInd++)
    {
        FeatureT PCLDescriptor;
        for(int ind = 0 ; ind < pDescriptors.cols; ind++)
        {
            PCLDescriptor.histogram[ind] = pDescriptors.at<unsigned char>(KeyPointsInd,ind);
        }
        FeaturePointCloud->points.push_back(PCLDescriptor);
    }
    return FeaturePointCloud;
}

std::vector<std::string> getFolderFileNames(std::string FolderName)
{
    FolderName+="/*";
    glob_t glob_result;
    glob(FolderName.c_str(),GLOB_TILDE,NULL,&glob_result);
    std::vector<std::string> ret;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        ret.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return ret;
}

std::vector<FrameData> readFolderOfImages(std::string FirstImageFileName , std::string FirstDepthFileName)
{
    std::vector<std::string> ret = getFolderFileNames(FirstImageFileName);
    std::vector<std::string> Dret = getFolderFileNames(FirstDepthFileName);

    std::vector<FrameData> frames ;
    for(size_t i = 0 ; i < ret.size(); i++){
        cv::Mat image = cv::imread(ret[i].c_str(), CV_LOAD_IMAGE_COLOR);
        cv::Mat Dimage = cv::imread(Dret[i].c_str(),CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        Dimage.convertTo(Dimage , CV_16U);
        if(image.data && Dimage.data)
        {
            frames.push_back(FrameData(image , ret[i],Dimage));
        }
    }
    return frames;
}

PointCloudT::Ptr GeneratePointCloud(cv::Mat pImage)
{
    PointCloudT::Ptr cloud (new PointCloudT);
    cloud->width = pImage.cols;
    cloud->height = pImage.rows;

    cloud->points.resize (cloud->width * cloud->height);

    float Z , factor = 5000;
    int i = 0 ;
    for(int rowInd = 0 ; rowInd < pImage.rows ; rowInd++)
        for(int colInd = 0 ; colInd < pImage.cols ; colInd++)
        {
            Z = pImage.at<float>(rowInd , colInd) / factor;
            cloud->points[i].x = ((colInd - cx) * Z )/fx;
            cloud->points[i].y = ((rowInd - cy) * Z )/fy;
            cloud->points[i].z = Z;
            i++;
        }
    return cloud;
}


void EstimateTransformationBetweenTwoConsecutiveFrames(PointCloudT::Ptr pCurrentPointCloud , PointCloudT::Ptr pPreviousPointCloud,
                                                       FeatureCloudT::Ptr pCurrentFeaturePointCloud , FeatureCloudT::Ptr pPreviousFeaturePointCloud)
{
    PointCloudT::Ptr object_aligned (new PointCloudT);
    const float leaf = 0.005f;

    pcl::SampleConsensusPrerejective< PointNT,PointNT,FeatureT > align;
    align.setInputSource (pCurrentPointCloud);
    align.setSourceFeatures (pCurrentFeaturePointCloud);
    align.setInputTarget (pPreviousPointCloud);
    align.setTargetFeatures (pPreviousFeaturePointCloud);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
    }
    if (align.hasConverged ())
    {
        // Print results
        //        printf ("\n");
        //        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        //        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        //        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        //        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        //        pcl::console::print_info ("\n");
        //        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        //        pcl::console::print_info ("\n");
        //        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

        //        // Show alignment
        //        pcl::visualization::PCLVisualizer visu("Alignment");
        //        visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
        //        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        //        visu.spin ();
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
    }
}
void EstimateTransformationBetweenTwoConsecutiveFramesSVD(PointCloudT::Ptr pCurrentPointCloud , PointCloudT::Ptr pPreviousPointCloud,
                                                       FeatureCloudT::Ptr pCurrentFeaturePointCloud , FeatureCloudT::Ptr pPreviousFeaturePointCloud ,  pcl::Correspondences &correspondences)
{
//    PointCloudT::Ptr object_aligned (new PointCloudT);
//    const float leaf = 0.005f;

//    pcl::registration::TransformationEstimationSVD<FeatureT,FeatureT> align;
//    Eigen::Matrix4f &transformation_matrix;
//    align.estimateRigidTransformation (pCurrentFeaturePointCloud, pPreviousFeaturePointCloud, correspondences , transformation_matrix);

}

void CorrespondenceRejectionUsingRANSAC()
{

}

void visualizePointCloud(FeatureCloudT::Ptr scene)
{
    //    pcl::visualization::PCLVisualizer visu("Alignment");
    //    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    //    visu.spin ();
}
int main ()
{
    //    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<FrameData> Frames = readFolderOfImages(rgbImages,depthImages);
    for(int i = 10 ; i < 11 ; i ++)
    {
        FrameData previousFrame = Frames[i-1];
        FrameData currentFrame = Frames[i];
        std::vector<cv::KeyPoint> CVCurrentKeypoints ,CVPreviousKeypoints ;
        cv::Mat CVCurrentDescriptors, CVPreviousDescriptors;

        computeFeatreBRISKFeatureDescriptor(previousFrame ,CVPreviousKeypoints , CVPreviousDescriptors );
        computeFeatreBRISKFeatureDescriptor(currentFrame ,CVCurrentKeypoints , CVCurrentDescriptors );
        //showDescriptor(CVCurrentDescriptors);

        matchTwoCVDescriptors(CVCurrentDescriptors , CVPreviousDescriptors , CVCurrentKeypoints, CVPreviousKeypoints, currentFrame , previousFrame);

        FeatureCloudT::Ptr PCLCurrentFeaturePointCloud = createFeaturePointCloud(CVCurrentDescriptors);
        FeatureCloudT::Ptr PCLPreviousFeaturePointCloud = createFeaturePointCloud(CVPreviousDescriptors);
        PointCloudT::Ptr PCLCurrentPointCloud = GeneratePointCloud(currentFrame.getFrameMatrix());
        PointCloudT::Ptr PCLPreviousPointCloud = GeneratePointCloud(previousFrame.getFrameMatrix());

        //        visualizePointCloud(PCLCurrentFeaturePointCloud);
//        matchTwoPCLDescriptors(PCLCurrentFeaturePointCloud , PCLPreviousFeaturePointCloud);
        //        std::cout << CVCurrentKeypoints.size() << " " << CVPreviousKeypoints.size() << std::endl;
        //        std::cout << CVCurrentDescriptors.rows << " " << CVCurrentDescriptors.cols << " "
        //        << CVPreviousDescriptors.rows << " " << CVPreviousDescriptors.cols << std::endl;
        //        std::cout << PCLCurrentFeaturePointCloud->points.size() << " " << PCLPreviousFeaturePointCloud->points.size()<<std::endl;

        //std::cout << "Estimating :" ;//<< std::to_string(i-1) << "and" << std::to_string(i) <<endl;
        EstimateTransformationBetweenTwoConsecutiveFrames(PCLCurrentPointCloud, PCLPreviousPointCloud , PCLCurrentFeaturePointCloud , PCLPreviousFeaturePointCloud);
    }

    return (0);
}

