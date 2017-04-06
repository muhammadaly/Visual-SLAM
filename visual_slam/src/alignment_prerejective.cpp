#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <visual_slam/framedata.h>
#include <glob.h>

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/pfh.h>

////#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZT;


static const float fx = 525.0;  // focal length x
static const float fy = 525.0;  // focal length y
static const float cx = 319.5;  // optical center x
static const float cy = 239.5;  // optical center y

std::string rgbImages = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/rgb/1305031526.671473.png";
std::string depthImages = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/depth/1305031526.688356.png";
std::string transformationMatrices = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/Results/UsingPCL/transformationMatrices.txt";
std::string detectionfilename = "/home/muhammadaly/Projects/TrajectoryEstimationForPCLAndOpenCV/Results/UsingPCL/Features/";
ofstream myfile;
using namespace pcl;

//void computeFeatreORBFeatureDescriptor(std::vector<FrameData> datasetImages)
//{
//    std::vector<int> compression_params;
//    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//    compression_params.push_back(9);
//    cv::Mat detectedFeature , img;
//    std::vector<cv::Mat> datasetDescriptors;
//    std::vector< std::vector<cv::KeyPoint> > datasetKeypoint ;
//    cv::Ptr<cv::ORB> orb = cv::ORB::create();
//
//    for(int i = 0 ; i < datasetImages.size() ; i++)
//    {
//        img = datasetImages[i].getFrameMatrix();
//        std::vector<cv::KeyPoint> tkeypoint ;
//        cv::Mat tdescriptors;
//
//        orb->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
//        datasetDescriptors.push_back(tdescriptors);
//        datasetKeypoint.push_back(tkeypoint);
////        drawKeypoints(img, tkeypoint , detectedFeature,Scalar(0,255,0), 0 );
//
////        imwrite(detectionfilename+ to_string(i)+".png", detectedFeature, compression_params);
//    }
//}
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

PointCloudT::Ptr GeneratePointCloud(cv::Mat currentDepthImage)
{
    PointCloudT::Ptr cloud (new PointCloudT);
    cloud->width = currentDepthImage.cols;
    cloud->height = currentDepthImage.rows;

    cloud->points.resize (cloud->width * cloud->height);

    float Z , factor = 5000;
    int i = 0 ;
    for(int rowInd = 0 ; rowInd < currentDepthImage.rows ; rowInd++)
        for(int colInd = 0 ; colInd < currentDepthImage.cols ; colInd++)
        {
            Z = currentDepthImage.at<float>(rowInd , colInd) / factor;
            cloud->points[i].x = ((colInd - cx) * Z )/fx;
            cloud->points[i].y = ((rowInd - cy) * Z )/fy;
            cloud->points[i].z = Z;
            i++;
        }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GeneratePointCloud3d(cv::Mat currentDepthImage)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //    cloud->width = currentDepthImage.cols;
    //    cloud->height = currentDepthImage.rows;

    //    cloud->points.resize (cloud->width * cloud->height);

    float Z , X , Y, factor = 5000;
    int i = 0 ;
    for(int rowInd = 0 ; rowInd < currentDepthImage.rows ; rowInd++)
        for(int colInd = 0 ; colInd < currentDepthImage.cols ; colInd++)
        {
            Z = currentDepthImage.at<float>(rowInd , colInd) / factor;
            X = ((colInd - cx) * Z )/fx;
            Y = ((rowInd - cy) * Z )/fy;

            pcl::PointXYZ p;
            p.x = X;
            p.y = Y;
            p.z = Z;
            cloud->points.push_back(p);
        }
    return cloud;
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
        cv::Mat image = cv::imread(ret[i], CV_LOAD_IMAGE_COLOR);
        cv::Mat Dimage = cv::imread(Dret[i],CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        Dimage.convertTo(Dimage , CV_16U);
        if(image.data && Dimage.data)
        {
            frames.push_back(FrameData(image , ret[i],Dimage));
        }
    }
    return frames;
}

void EstimateTransformationBetweenTwoConsecutiveFrames(FrameData previousFrame , FrameData currentFrame)
{
    // Align a rigid object to a scene with clutter and occlusions

    cv::Mat previousImage = previousFrame.getDepthMatrix();
    cv::Mat currentImage = currentFrame.getDepthMatrix();
    //    namedWindow( "previous Image", WINDOW_AUTOSIZE );
    //    imshow( "previous Image", previousImage );
    //    namedWindow( "current Image", WINDOW_AUTOSIZE );
    //    imshow( "current Image", currentImage );

    //    waitKey(0);
    std::string currentTimeStamp = previousFrame.getTimestamp();

    //pcl::PointCloud<pcl::PointXYZ>::Ptr object3d = GeneratePointCloud3d(previousImage);
    PointCloudT::Ptr object =  GeneratePointCloud(previousImage) ;// (new PointCloudT);
    PointCloudT::Ptr object_output (new PointCloudT);
    PointCloudT::Ptr object_aligned (new PointCloudT);
    PointCloudT::Ptr scene  = GeneratePointCloud(currentImage) ;// (new PointCloudT);
    PointCloudT::Ptr scene_downsampled (new PointCloudT);
    PointCloudT::Ptr object_downsampled (new PointCloudT);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr scene3d = GeneratePointCloud3d(currentImage);
    PointCloudT::Ptr scene_output (new PointCloudT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    // Downsample
    pcl::console::print_highlight ("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 5.0f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(object);
    grid.filter(*object_downsampled);
    grid.setInputCloud(scene);
    grid.filter(*scene_downsampled);

    // remove NAN points
    std::vector<int> object_indices;
    pcl::removeNaNFromPointCloud(*object_downsampled, *object_output, object_indices);
    std::vector<int> scene_indices;
    pcl::removeNaNFromPointCloud(*scene_downsampled, *scene_output, scene_indices);

    // Estimate normals for scene
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    //    pcl::search::KdTree<PointNT>::Ptr tree (new pcl::search::KdTree<PointNT> ());
    //    nest.setSearchMethod (tree);
    nest.setRadiusSearch(0.03);
    nest.setInputCloud (object_output);
    //nest.setSearchSurface(object);
    nest.compute (*object_output);
    nest.setInputCloud (scene_output);
    //nest.setSearchSurface(scene);
    nest.compute (*scene_output);


    // Estimate features
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.25);
    fest.setInputCloud(object_output);
    fest.setInputNormals(object_output);
    fest.compute(*object_features);
    fest.setInputCloud(scene_output);
    fest.setInputNormals(scene_output);
    fest.compute(*scene_features);

    //    pcl::PFHEstimation<PointNT, PointNT, pcl::PFHSignature125> pfh;
    //    pcl::search::KdTree<PointNT>::Ptr tree (new pcl::search::KdTree<PointNT> ());
    //    pfh.setSearchMethod (tree);
    //    pfh.setInputCloud (object_output);
    //    pfh.setInputNormals (object_output);
    //    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhsObject (new pcl::PointCloud<pcl::PFHSignature125> ());
    //    pfh.setRadiusSearch (0.05);
    //    pfh.compute (*pfhsObject);
    //    pfh.setInputCloud (scene_output);
    //    pfh.setInputNormals (scene_output);
    //    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhsScene (new pcl::PointCloud<pcl::PFHSignature125> ());
    //    pfh.compute (*pfhsScene);



    // Perform alignment
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<pcl::PointNormal,pcl::PointNormal,FeatureT> align;
    align.setInputSource (object_output);
    align.setSourceFeatures (object_features);
    align.setInputTarget (scene_output);
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
        //        // Print results
        //        printf ("\n");
        //        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        //        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        //        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        //        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        //        pcl::console::print_info ("\n");
        //        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        //        pcl::console::print_info ("\n");
        //        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

        // Show alignment
        //pcl::visualization::PCLVisualizer visu("Alignment");
        //visu.addPointCloud (scene3d, ColorHandlerXYZT (scene3d, 0.0, 255.0, 0.0), "scene");
        //visu.addPointCloud (object3d, ColorHandlerXYZT (object3d, 0.0, 255.0, 255.0), "scene");
        //visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        //visu.spin ();
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        writeResultFile(transformation , currentTimeStamp);
    }
    else
    {

    }
}

int
main ()
{
    pcl::console::print_highlight ("Reading Frames ...\n");
    std::vector<FrameData> Frames = readFolderOfImages(rgbImages,depthImages);

    for(int i = 10 ; i < Frames.size() ; i ++)
    {
        FrameData previousFrame = Frames[i-1];
        FrameData currentFrame = Frames[i];
        //cout << "Estimating :" << to_string(i-1) << "and" << to_string(i) <<endl;
        EstimateTransformationBetweenTwoConsecutiveFrames(previousFrame, currentFrame);
    }

    return (0);
}

