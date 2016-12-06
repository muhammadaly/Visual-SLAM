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
#include <pcl/common/pca.h>

static const int ORBFeatureVectorLength = 32;
static const int BRISKFeatureVectorLength = 64;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

float fx = 525.0;  // focal length x
float fy = 525.0;  // focal length y
float cx = 319.5;  // optical center x
float cy = 239.5;  // optical center y

std::string datasetDIR = "/home/muhammadaly/master_dataset/rgbd_dataset_freiburg1_xyz/";
std::string rgbImages = datasetDIR + "rgb/";
std::string depthImages = datasetDIR + "depth/";
std::string transformationMatrices = datasetDIR + "transformationMatricesPCL.txt";
std::string featureMatching = datasetDIR + "matching/";
std::string frames_matching = datasetDIR + "matching_frames.txt";

std::ofstream myfile;

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

void showImage(cv::Mat img)
{
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", img );                   // Show our image inside it.

    cv::waitKey(0);
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

std::vector<FrameData> readDataset()
{
    std::vector<FrameData> frames ;
    std::string line, tmp;
    char * pch;
    std::vector<std::string> depthFiles , rgbFiles ;
    std::ifstream myfile (frames_matching.c_str());
    if (myfile.is_open())
    {
        while ( std::getline(myfile,line) )
        {
            pch = std::strtok(const_cast<char*>(line.c_str())," /");
            int i = 0;
            while (pch != NULL)
            {
                if (i == 2)
                {
                    tmp = std::string(pch);
                    rgbFiles.push_back(tmp);
                }
                else if (i == 5)
                {
                    tmp = std::string(pch);
                    depthFiles.push_back(tmp);
                }
                pch = strtok (NULL, " /");
                i++;
            }
        }
        for(int i = 0 ; i < rgbFiles.size() ; i++)
        {
            cv::Mat image = cv::imread((rgbImages+rgbFiles[i]).c_str());
            cv::Mat Dimage = cv::imread((depthImages+depthFiles[i]).c_str(),CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
            Dimage.convertTo(Dimage , CV_16U);
            if(image.data && Dimage.data)
                frames.push_back(FrameData(image , (depthImages+depthFiles[i]).c_str() ,Dimage));
        }
        myfile.close();

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
            PointNT p;
            p.x = ((colInd - cx) * Z )/fx;
            p.y = ((rowInd - cy) * Z )/fy;
            p.z = Z;

            cloud->points.push_back(p);
        }
    // visualizePointCloud(cloud);
    return cloud;
}

int main ()
{
    std::vector<FrameData> Frames = readDataset();
    for(int i = 1 ; i < Frames.size() ; i ++)
    {
        FrameData previousFrame = Frames[i-1];
        FrameData currentFrame = Frames[i];
        cv::Mat previousImg = previousFrame.getDepthMatrix();
        cv::Mat currentImg = currentFrame.getDepthMatrix();

        PointCloudT::Ptr scene = GeneratePointCloud(previousImg);
        PointCloudT::Ptr object = GeneratePointCloud(currentImg);
        PointCloudT::Ptr object_aligned (new PointCloudT);
        FeatureCloudT::Ptr object_features (new FeatureCloudT);
        FeatureCloudT::Ptr scene_features (new FeatureCloudT);

//        pcl::visualization::PCLVisualizer visu("Alignment");
//        visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//        visu.addPointCloud (object_aligned, ColorHandlerT (object, 0.0, 0.0, 255.0), "object_aligned");
//        visu.spin ();

        // Downsample
        pcl::console::print_highlight ("Downsampling...\n");
        pcl::VoxelGrid<PointNT> grid;
        const float leaf = 0.09f;
        grid.setLeafSize (leaf, leaf, leaf);
        grid.setInputCloud (object);
        grid.filter (*object);
        grid.setInputCloud (scene);
        grid.filter (*scene);

        // Estimate normals for scene
        pcl::console::print_highlight ("Estimating scene normals...\n");
        pcl::NormalEstimationOMP<PointNT,PointNT> nest;
        nest.setRadiusSearch (0.01);
        nest.setInputCloud (scene);
        nest.compute (*scene);

        // Estimate features
        pcl::console::print_highlight ("Estimating features...\n");
        FeatureEstimationT fest;
        fest.setRadiusSearch (0.025);
        fest.setInputCloud (object);
        fest.setInputNormals (object);
        fest.compute (*object_features);
        fest.setInputCloud (scene);
        fest.setInputNormals (scene);
        fest.compute (*scene_features);

        // Perform alignment
        pcl::console::print_highlight ("Starting alignment...\n");
        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
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
            printf ("\nDone\n");
            Eigen::Matrix4f transformation = align.getFinalTransformation ();
            std::string currentTimeStamp = previousFrame.getTimestamp();
            writeResultFile(transformation,currentTimeStamp);
        }
        else
        {
            pcl::console::print_error ("Alignment failed!\n");
        }
    }
}


