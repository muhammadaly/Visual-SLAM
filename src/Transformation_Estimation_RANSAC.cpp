#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <visual_slam/framedata.h>
#include <glob.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

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
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

static const int ORBFeatureVectorLength = 32;
static const int BRISKFeatureVectorLength = 64;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Histogram<BRISKFeatureVectorLength> FeatureT;
//typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;

float fx = 525.0;  // focal length x
float fy = 525.0;  // focal length y
float cx = 319.5;  // optical center x
float cy = 239.5;  // optical center y

std::string rgbImages = "/home/muhammadaly/master dataset/rgbd_dataset_freiburg1_xyz/rgb";
std::string depthImages = "/home/muhammadaly/master dataset/rgbd_dataset_freiburg1_xyz/depth";
std::string transformationMatrices = "/home/muhammadaly/master dataset/Results/UsingPCL/transformationMatrices.txt";
std::string featureMatching = "/home/muhammadaly/master dataset/rgbd_dataset_freiburg1_xyz/matching";

std::ofstream myfile;

struct BRISKFeature
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float Histogram;
} BRISK_XYZ;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (BRISKFeature,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float , Histogram, Histogram)
                                   )

void writeResultFile(Eigen::Matrix4f transformation , std::string timestamp)
{
    myfile.open (transformationMatrices.c_str() , std::ios_base::app);
    myfile << timestamp << " " ;
    myfile << round(transformation (0,3) * 10000.0 ) / 10000.0  <<  " ";
    myfile << round(transformation (1,3) * 10000.0 ) / 10000.0  <<  " ";
    myfile << round(transformation (2,3) * 10000.0 ) / 10000.0  <<  " ";

    double qw = sqrt(1.0 + transformation(0,0) + transformation(1,1) + transformation(2,2)) / 2.0;
    double w4 = 4.0 * qw ;
    double qx = (transformation(2,1) - transformation(1,2)) / w4;
    double qy = (transformation(0,2) - transformation(2,0)) / w4;
    double qz = (transformation(1,0) - transformation(0,1)) / w4;

    myfile << round( qx * 10000.0 ) / 10000.0 << " " <<
              round( qy * 10000.0 ) / 10000.0 << " " <<
              round( qz * 10000.0 ) / 10000.0 << " " <<
              round( qw * 10000.0 ) / 10000.0 << "\n";
    myfile.close();
}


void visualizePointCloud(PointCloudT::Ptr scene, PointCloudT::Ptr object)
{
    pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
    visu.addPointCloud(scene,ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud(object,ColorHandlerT(object, 0.0, 0.0, 255.0), "object");
    visu.spin();
}
void visualizePointCloud(PointCloudT::Ptr scene)
{
    pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
    visu.addPointCloud(scene,ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    visu.spin();
}

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

std::vector<cv::DMatch> matchTwoCVDescriptors(cv::Mat currentImageDescriptors , cv::Mat previousImageDescriptors
                                              , std::vector<cv::KeyPoint> currentImageKeypoints , std::vector<cv::KeyPoint> previousImageKeypoints
                                              , FrameData currentFrame, FrameData previousFrame)
{
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;

    std::vector<cv::DMatch> good_matches;

    if (!currentImageDescriptors.empty() && !previousImageDescriptors.empty()) {
        if (currentImageDescriptors.type() != CV_32F) {
            currentImageDescriptors.convertTo(currentImageDescriptors, CV_32F);
        }
        if (previousImageDescriptors.type() != CV_32F) {
            previousImageDescriptors.convertTo(previousImageDescriptors, CV_32F);
        }
        matcher.match(currentImageDescriptors, previousImageDescriptors, matches);

        //        printf("Keypoints size : %i\n , Descriptor size : %i , %i \n" , currentImageKeypoints.size() , currentImageDescriptors.rows , currentImageDescriptors.cols);
        //        double max_dist = 0;
        //        double min_dist = 100;

        //        for (int i = 0; i < matches.size(); i++) {
        //            double dist = matches[i].distance;
        //            if (dist < min_dist) min_dist = dist;
        //            if (dist > max_dist) max_dist = dist;
        //        }

        //        for (int i = 0; i < matches.size(); i++) {
        //            if (matches[i].distance < 10 * min_dist) { good_matches.push_back(matches[i]); }
        //        }

        //        std::cout << matches.size() << std::endl;
    }
    return matches;
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
FeatureCloudT::Ptr createFeaturePointCloud( cv::Mat pDescriptors , std::vector<cv::DMatch> good_matches , bool current)
{
    FeatureCloudT::Ptr FeaturePointCloud ( new FeatureCloudT);
    //    printf("rows : %i , cols : %i\n" , pDescriptors.rows , pDescriptors.cols);
    for(int matchInd = 0 ; matchInd < good_matches.size() ; matchInd++)
    {
        FeatureT PCLDescriptor;
        if(current)
        {
            for(int ind = 0 ; ind < pDescriptors.cols; ind++)
            {
                PCLDescriptor.histogram[ind] = pDescriptors.at<unsigned char>(good_matches[matchInd].queryIdx,ind);
            }
        }
        else {
            for(int ind = 0 ; ind < pDescriptors.cols; ind++)
            {
                PCLDescriptor.histogram[ind] = pDescriptors.at<unsigned char>(good_matches[matchInd].trainIdx,ind);
            }
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
        cv::Mat image = cv::imread(ret[i].c_str());//, CV_LOAD_IMAGE_COLOR);
        cv::Mat Dimage = cv::imread(Dret[i].c_str(),CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        Dimage.convertTo(Dimage , CV_16U);
        //cv::Mat Dimage = cv::imread(Dret[i].c_str());//, CV_LOAD_IMAGE_COLOR);
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

    float Z , factor = 5000;
    for(int rowInd = 0 ; rowInd < pImage.rows ; rowInd++)
        for(int colInd = 0 ; colInd < pImage.cols ; colInd++)
        {
            Z = pImage.at<u_int16_t>(rowInd , colInd) / factor;
            PointT p;
            p.x = ((colInd - cx) * Z )/fx;
            p.y = ((rowInd - cy) * Z )/fy;
            p.z = Z;

            cloud->points.push_back(p);
        }
    visualizePointCloud(cloud);
    return cloud;
}

pcl::CorrespondencesPtr generatePCLcorrespondance(std::vector< cv::DMatch > CVmatches)
{
    pcl::CorrespondencesPtr generatedCorrespondences ( new pcl::Correspondences) ;
    for(int ind = 0 ; ind < CVmatches.size() ; ind ++ )
    {
        pcl::Correspondence c ( CVmatches[ind].queryIdx ,CVmatches[ind].trainIdx , CVmatches[ind].distance );
        generatedCorrespondences->push_back(c);
    }
    return generatedCorrespondences;
}
void EstimateTransformationBetweenTwoConsecutiveFramesSVD(PointCloudT::Ptr pCurrentPointCloud
                                                          , PointCloudT::Ptr pPreviousPointCloud, pcl::Correspondences correspondences
                                                          , std::string currentTimeStamp)
{
    //PointCloudT::Ptr object_aligned (new PointCloudT);
    //const float leaf = 0.005f;

    pcl::registration::TransformationEstimationSVD<PointT,PointT,float> align;
    pcl::registration::TransformationEstimationSVD<PointT,PointT,float>::Matrix4  transformation_matrix;
    align.estimateRigidTransformation (*pCurrentPointCloud, *pPreviousPointCloud, correspondences , transformation_matrix);

    std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", transformation_matrix (0,0), transformation_matrix (0,1), transformation_matrix (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", transformation_matrix (1,0), transformation_matrix (1,1), transformation_matrix (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", transformation_matrix (2,0), transformation_matrix (2,1), transformation_matrix (2,2));
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_matrix (0,3), transformation_matrix (1,3),transformation_matrix (2,3));
    //writeResultFile(transformation_matrix,currentTimeStamp);

}

void EstimateTransformationBetweenTwoConsecutiveFrames(PointCloudT::Ptr pCurrentPointCloud
                                                       , PointCloudT::Ptr pPreviousPointCloud)
{
    PointCloudT::Ptr cloud_source_registered(new PointCloudT);
    pcl::IterativeClosestPoint<PointT, PointT>::Ptr align ( new pcl::IterativeClosestPoint<PointT, PointT>);
    align->setInputCloud(pCurrentPointCloud);
    align->setInputTarget(pPreviousPointCloud);
    align->setMaxCorrespondenceDistance (0.05);
    align->setMaximumIterations (50);
    align->setTransformationEpsilon (1e-8);
    align->setEuclideanFitnessEpsilon (1);
    align->align(*cloud_source_registered);

    Eigen::Matrix4f transformation_matrix = align->getFinalTransformation ();
    std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", transformation_matrix (0,0), transformation_matrix (0,1), transformation_matrix (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", transformation_matrix (1,0), transformation_matrix (1,1), transformation_matrix (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", transformation_matrix (2,0), transformation_matrix (2,1), transformation_matrix (2,2));
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_matrix (0,3), transformation_matrix (1,3),transformation_matrix (2,3));

}
void showImageData(cv::Mat image)
{
    float d;
    for(int y = 0; y < image.rows ; y++)
    {
        for(int x = 0; x < image.cols ; x++)
        {
            d = image.at<float>(y,x);
            if(d!=0.0)
                printf("%f ",d);
        }
        printf("\n");
    }
}
void generateSelectedPointCloud(std::vector<cv::DMatch> matches , std::vector<cv::KeyPoint> previousKeypoints , std::vector<cv::KeyPoint> currentKeypoints
                                , FrameData previousFrameData , FrameData currentFrameData ,PointCloudT::Ptr previousSelectedPointCloud , PointCloudT::Ptr currentSelectedPointCloud  )
{
    float Z , factor = 5000;

    //cv::Mat currentImage = currentFrameData.getFrameMatrix();
    //cv::Mat previousImage = previousFrameData.getFrameMatrix();
    cv::Mat currentDepthImage = currentFrameData.getDepthMatrix();
    cv::Mat previousDepthImage = previousFrameData.getDepthMatrix();

    cv::KeyPoint currentKeypoint , previousKeypoint;
    int rowInd , colInd;
    for(size_t ind = 1 ;ind < matches.size() ; ind++)
    {
        currentKeypoint = currentKeypoints[matches[ind].queryIdx];
        previousKeypoint = previousKeypoints[matches[ind].trainIdx];

        rowInd = previousKeypoint.pt.y;
        colInd = previousKeypoint.pt.x;
        Z = previousDepthImage.at<u_int16_t>(rowInd , colInd) / factor;

        PointT p;
        p.x = ((colInd - cx) * Z )/fx;
        p.y = ((previousKeypoint.pt.y - cy) * Z )/fy;
        p.z = Z;
        previousSelectedPointCloud->points.push_back(p);

        rowInd = currentKeypoint.pt.y;
        colInd = currentKeypoint.pt.x;

        Z = currentDepthImage.at<u_int16_t>(rowInd , colInd) / factor;

        PointT p2;
        p2.x = ((colInd - cx) * Z )/fx;
        p2.y = ((rowInd - cy) * Z )/fy;
        p2.z = Z;
        currentSelectedPointCloud->points.push_back(p2);

    }
    //        visualizePointCloud(previousSelectedPointCloud , currentSelectedPointCloud);
}


void OutliersRejection(PointCloudT::Ptr pCurrentPointCloud, PointCloudT::Ptr pPreviousPointCloud)
{
    std::vector<int> IdxInliers;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr RANSAC(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>);
    RANSAC->setInputSource(pPreviousPointCloud);
    RANSAC->setInputTarget(pCurrentPointCloud);
    RANSAC->setMaximumIterations(5000);
    RANSAC->setInlierThreshold(0.05);
    RANSAC->getInliersIndices(IdxInliers);
    printf("IdxInliers size %i\n", ((int)IdxInliers.size()));
}


int main ()
{
    //    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<FrameData> Frames = readFolderOfImages(rgbImages,depthImages);
    //for(int i = 1 ; i < Frames.size() ; i ++)
    for(int i = 200 ; i < 201 ; i ++)
    {
        FrameData previousFrame = Frames[i-1];
        FrameData currentFrame = Frames[i];
        //                cv::Mat previousDepth = previousFrame.getDepthMatrix();
        //                GeneratePointCloud(previousDepth);
        std::vector<cv::KeyPoint> CVCurrentKeypoints ,CVPreviousKeypoints ;
        cv::Mat CVCurrentDescriptors, CVPreviousDescriptors , goodCVCurrentDescriptors, goodCVPreviousDescriptors;

        computeFeatreBRISKFeatureDescriptor(previousFrame ,CVPreviousKeypoints , CVPreviousDescriptors );
        computeFeatreBRISKFeatureDescriptor(currentFrame ,CVCurrentKeypoints , CVCurrentDescriptors );

        std::vector<cv::DMatch> good_matches = matchTwoCVDescriptors(CVCurrentDescriptors , CVPreviousDescriptors , CVCurrentKeypoints, CVPreviousKeypoints, currentFrame , previousFrame);
        printf("good_matches size %i\n" , ((int)good_matches.size()));

        PointCloudT::Ptr CurrentSelectedPointCloud (new PointCloudT), PreviousSelectedPointCloud (new PointCloudT);
        generateSelectedPointCloud(good_matches,CVPreviousKeypoints,CVCurrentKeypoints,previousFrame , currentFrame
                                   ,PreviousSelectedPointCloud ,CurrentSelectedPointCloud);
//        OutliersRejection(CurrentSelectedPointCloud, PreviousSelectedPointCloud);
        EstimateTransformationBetweenTwoConsecutiveFrames(CurrentSelectedPointCloud,PreviousSelectedPointCloud);

    }

    return (0);
}

