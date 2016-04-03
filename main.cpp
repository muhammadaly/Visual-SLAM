#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <glob.h>
#include <math.h>
#include <iomanip>
#include <framedata.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void readme();
ofstream myfile;
void RenderCVMatrix(cv::Mat pMat , int numCol , int numRow)
{
    for(int i = 0 ; i < pMat.rows && i< numRow ; i++)
    {
        for(int j = 0 ; j < pMat.cols && j < numCol ; j++)
            cout << pMat.at<double>(i,j) <<endl;
    }
}
vector<FrameData> readFolderOfImages(std::string FirstImageFileName , std::string FirstdepthImage)
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

    std::size_t Dfound = FirstdepthImage.find_last_of("/\\");
    std::string Dfolder = FirstdepthImage.substr(0,Dfound);
    Dfolder+="/*";

    glob_t Dglob_result;
    glob(Dfolder.c_str(),GLOB_TILDE,NULL,&Dglob_result);
    std::vector<std::string> Dret;
    for(unsigned int i=0;i<Dglob_result.gl_pathc;++i){
        Dret.push_back(std::string(Dglob_result.gl_pathv[i]));
    }
    globfree(&Dglob_result);

    vector<FrameData> frames ;
    for(size_t i = 0 ; i < ret.size()-1; i++){
        cv::Mat image = cv::imread(ret[i], CV_LOAD_IMAGE_COLOR);
        cv::Mat Dimage = cv::imread(Dret[i],CV_LOAD_IMAGE_ANYDEPTH);
        if(image.data )
        {
            frames.push_back(FrameData(image , ret[i],Dimage));
        }
    }
    return frames;
}

void writeResultFile(Mat R , Mat T , std::string timestamp)
{

//    w = Math.sqrt(1.0 + m1.m00 + m1.m11 + m1.m22) / 2.0;
//    double w4 = (4.0 * w);
//    x = (m1.m21 - m1.m12) / w4 ;
//    y = (m1.m02 - m1.m20) / w4 ;
//    z = (m1.m10 - m1.m01) / w4 ;
    myfile << timestamp << " " ;
    for(int i = 0 ; i< T.rows ; i++)
        myfile <<  round( T.at<double>(i,0) * 10000.0 ) / 10000.0  << ' ';
    double qw = sqrt(1.0 + R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2)) / 2.0;
    double w4 = 4.0 * qw ;
    double qx = (R.at<double>(2,1) - R.at<double>(1,2)) / w4;
    double qy = (R.at<double>(0,2) - R.at<double>(2,0)) / w4;
    double qz = (R.at<double>(1,0) - R.at<double>(0,1)) / w4;

    myfile << round( qx * 10000.0 ) / 10000.0 << ' ' <<
              round( qy * 10000.0 ) / 10000.0 << ' ' <<
              round( qz * 10000.0 ) / 10000.0 << ' ' <<
              round( qw * 10000.0 ) / 10000.0 << '\n';
}

int main(int argc, char* argv[])
{
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    std::string folderName = "/media/muhammadaly/2DE6-372E/Thesis Work/RGB D Dataset/rgbd_dataset_freiburg1_desk2/";
    std::string RGBfilename= folderName+"rgb/1305031102.175304.png";
    std::string Depthfilename = folderName+ "depth/1305031102.160407.png";
    std::string detectionfilename = folderName+"rgb-Detection";
    std::string matchingfilename = folderName+"rgb-Matching/";
    std::string transformationMatrices = folderName+"transformationMatrices.txt";
    vector<FrameData> datasetImages = readFolderOfImages(RGBfilename,Depthfilename);
    Ptr<ORB> orb = ORB::create();

    Mat detectedFeature , img;
    vector<Mat> datasetDescriptors;
    vector<vector<KeyPoint>> datasetKeypoint ;

    for(int i = 0 ; i < datasetImages.size() ; i++)
    {
        img = datasetImages[i].getFrameMatrix();
        vector<KeyPoint> tkeypoint ;
        Mat tdescriptors;

        orb->detectAndCompute(img, noArray(),tkeypoint, tdescriptors);
        datasetDescriptors.push_back(tdescriptors);
        datasetKeypoint.push_back(tkeypoint);
        //drawKeypoints(img, tkeypoint , detectedFeature,Scalar(0,255,0), 0 );

        //imwrite(detectionfilename+ to_string(i)+".png", detectedFeature, compression_params);
    }

    Mat currentImageDescriptors , previousImageDescriptors ,
            currentImage , previousImage , currentDepthImage , previousDepthImage;
    vector<KeyPoint> currentImageKeypoints , previousImageKeypoints;
    FrameData currentFrameData , previousFrameData ;
    myfile.open (transformationMatrices);

    for(int i = 1 ; i < datasetDescriptors.size() ; i++)
    {
        currentFrameData = datasetImages[i];
        previousFrameData = datasetImages[i-1];
        currentImageDescriptors = datasetDescriptors[i];
        previousImageDescriptors = datasetDescriptors[i-1];
        currentImage = datasetImages[i].getFrameMatrix();
        previousImage = datasetImages[i-1].getFrameMatrix();
        currentDepthImage = datasetImages[i].getDepthMatrix();
        previousDepthImage = datasetImages[i-1].getDepthMatrix();;
        currentImageKeypoints = datasetKeypoint[i];
        previousImageKeypoints = datasetKeypoint[i-1];

        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        if(! currentImageDescriptors.empty() && !previousImageDescriptors.empty())
        {
            if(currentImageDescriptors.type()!=CV_32F) {
                currentImageDescriptors.convertTo(currentImageDescriptors, CV_32F);
            }
            if(previousImageDescriptors.type()!=CV_32F) {
                previousImageDescriptors.convertTo(previousImageDescriptors, CV_32F);
            }
            matcher.match( currentImageDescriptors, previousImageDescriptors, matches );
            double max_dist = 0; double min_dist = 100;

            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < currentImageDescriptors.rows; i++ )
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            std::vector< DMatch > good_matches;

            for( int i = 0; i < currentImageDescriptors.rows; i++ )
            { if( matches[i].distance < 6*min_dist )
                { good_matches.push_back( matches[i]); }
            }
            vector<Point2f> currentImagePoints , previousImagePoints ;
            double focal = 517.3;
            Point2f pp(0.0, 0.0);
            for(int i = 0 ; i < matches.size() ; i++)
            {
                Point2f point1 = currentImageKeypoints[matches[i].queryIdx].pt;
                currentImagePoints.push_back(point1);
                Point2f point2 = previousImageKeypoints[matches[i].trainIdx].pt;
                previousImagePoints.push_back(point2);
            }

            Mat E , mask , R , T;
            E = findEssentialMat(currentImagePoints , previousImagePoints , focal ,pp,RANSAC , 0.999,1,mask);
            recoverPose(E, currentImagePoints, previousImagePoints, R, T, focal, pp, mask);
            writeResultFile(R , T , currentFrameData.getTimestamp() );
            //            Mat img_matches;
            //            drawMatches( currentImage, currentImageKeypoints, previousImage, previousImageKeypoints,
            //                         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
            //                         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            //            imwrite(matchingfilename+ to_string(i-1)+"_"+to_string(i)+".png", img_matches, compression_params);
            int x = 0;
        }

    }
    myfile.close();

}
