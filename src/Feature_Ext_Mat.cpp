#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <visual_slam/framedata.h>
#include <glob.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

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

static const int ORBFeatureVectorLength = 35;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::Histogram<ORBFeatureVectorLength> FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

static const float fx = 525.0;  // focal length x
static const float fy = 525.0;  // focal length y
static const float cx = 319.5;  // optical center x
static const float cy = 239.5;  // optical center y

std::string rgbImages = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/rgb/1305031526.671473.png";
std::string depthImages = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/depth/1305031526.688356.png";
std::string transformationMatrices = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/Results/UsingPCL/transformationMatrices.txt";
std::string detectionfilename = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/Results/UsingPCL/Features/";

std::ofstream myfile;
void computeFeatreORBFeatureDescriptor(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors , cv::Ptr<cv::ORB> orb)
{
    cv::Mat img;

    img = pFrameData.getFrameMatrix();
    orb->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
}
void matchTwoCVDescriptors(cv::Mat currentImageDescriptors , cv::Mat previousImageDescriptors)
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

        std::cout << good_matches.size() << std::endl;
        for(int i = 0 ; i < good_matches.size() ; i++)
        {
            std::cout << good_matches[i].trainIdx << " " << good_matches[i].queryIdx << std::endl;
        }

    }
}
FeatureCloudT::Ptr createFeaturePointCloud( cv::Mat pDescriptors)
{
    FeatureCloudT::Ptr FeaturePointCloud ( new FeatureCloudT);

    for(int KeyPointsInd = 0 ; KeyPointsInd < pDescriptors.rows ; KeyPointsInd++)
    {
        FeatureT PCLDescriptor;
        for(int ind = 0 ; ind < pDescriptors.cols; ind++)
        {
            PCLDescriptor.histogram[ind] = pDescriptors.at<float>(KeyPointsInd,ind);
        }
        FeaturePointCloud->points.push_back(PCLDescriptor);
    }
    return FeaturePointCloud;
}

std::vector<FrameData> readFolderOfImages(std::string FirstImageFileName , std::string FirstDepthFileName)
{
    std::size_t found = FirstImageFileName.find_last_of("/\\");
    std::string folder = FirstImageFileName.substr(0,found);
    folder+="/*";

    glob_t glob_result;
    glob(folder.c_str(),GLOB_TILDE,NULL,&glob_result);
    std::vector<std::string> ret;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        ret.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);

    std::size_t Dfound = FirstDepthFileName.find_last_of("/\\");
    std::string Dfolder = FirstDepthFileName.substr(0,Dfound);
    Dfolder+="/*";

    glob_t Dglob_result;
    glob(Dfolder.c_str(),GLOB_TILDE,NULL,&Dglob_result);
    std::vector<std::string> Dret;
    for(unsigned int i=0;i<Dglob_result.gl_pathc;++i){
        Dret.push_back(std::string(Dglob_result.gl_pathv[i]));
    }
    globfree(&Dglob_result);

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

int
main ()
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<FrameData> Frames = readFolderOfImages(rgbImages,depthImages);
    for(int i = 1 ; i < 2 ; i ++)
    {
        FrameData previousFrame = Frames[i-1];
        FrameData currentFrame = Frames[i];
        std::vector<cv::KeyPoint> CVCurrentKeypoints ,CVPreviousKeypoints ;
        cv::Mat CVCurrentDescriptors, CVPreviousDescriptors;

        computeFeatreORBFeatureDescriptor(previousFrame ,CVPreviousKeypoints , CVPreviousDescriptors , orb );
        computeFeatreORBFeatureDescriptor(currentFrame ,CVCurrentKeypoints , CVCurrentDescriptors , orb );

        matchTwoCVDescriptors(CVCurrentDescriptors , CVPreviousDescriptors);

        FeatureCloudT::Ptr PCLCurrentFeaturePointCloud = createFeaturePointCloud(CVCurrentDescriptors);
        FeatureCloudT::Ptr PCLPreviousFeaturePointCloud = createFeaturePointCloud(CVPreviousDescriptors);

//        std::cout << CVCurrentKeypoints.size() << " " << CVPreviousKeypoints.size() << std::endl;
//        std::cout << CVCurrentDescriptors.rows << " " << CVCurrentDescriptors.cols << " "
//        << CVPreviousDescriptors.rows << " " << CVPreviousDescriptors.cols << std::endl;
//        std::cout << PCLCurrentFeaturePointCloud->points.size() << " " << PCLPreviousFeaturePointCloud->points.size()<<std::endl;

        //cout << "Estimating :" << to_string(i-1) << "and" << to_string(i) <<endl;
        //EstimateTransformationBetweenTwoConsecutiveFrames(previousFrame, currentFrame);
    }

    return (0);
}

