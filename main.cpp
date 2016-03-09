
#include <iostream>
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

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void readme();

vector<Mat> readFolderOfImages(std::string FirstImageFileName)
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
    vector<Mat> frames ;
    for(size_t i = 0 ; i < ret.size(); i++){
        cv::Mat image = cv::imread(ret[i], CV_LOAD_IMAGE_COLOR);
        if(image.data )
        {
            frames.push_back(image);
        }
    }
    return frames;
}

int main(int argc, char* argv[])
{
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    std::string filename= "/home/muhammadaly/Dataset/rgb/1305031102.175304.png";
    std::string detectionfilename = "/home/muhammadaly/Dataset/rgb-Detection";
    std::string matchingfilename = "/home/muhammadaly/Dataset/rgb-Matching";
    vector<Mat> datasetImages = readFolderOfImages(filename);
    Ptr<ORB> orb = ORB::create();

    Mat detectedFeature , img;
    vector<Mat> datasetDescriptors;
    vector<vector<KeyPoint>> datasetKeypoint ;

    for(int i = 0 ; i < datasetImages.size() ; i++)
    {
        img = datasetImages[i];
        vector<KeyPoint> tkeypoint ;
        Mat tdescriptors;

        orb->detectAndCompute(img, noArray(),tkeypoint, tdescriptors);
        datasetDescriptors.push_back(tdescriptors);
        datasetKeypoint.push_back(tkeypoint);
        //drawKeypoints(img, tkeypoint , detectedFeature,Scalar(0,255,0), 0 );

        //imwrite(detectionfilename+ to_string(i)+".png", detectedFeature, compression_params);
    }

//    Mat currentImageDescriptors , previousImageDescriptors ,
//            currentImage , previousImage;
//    vector<KeyPoint> currentImageKeypoints , previousImageKeypoints;
//    for(int i = 1 ; i < datasetDescriptors.size() ; i++)
//    {
//        currentImageDescriptors = datasetDescriptors[i];
//        previousImageDescriptors = datasetDescriptors[i-1];
//        currentImage = datasetImages[i];
//        previousImage = datasetImages[i-1];
//        currentImageKeypoints = datasetKeypoint[i];
//        previousImageKeypoints = datasetKeypoint[i-1];

//        FlannBasedMatcher matcher;
//        std::vector< DMatch > matches;
//        matcher.match( currentImageDescriptors, previousImageDescriptors, matches );
//        double max_dist = 0; double min_dist = 100;

//        //-- Quick calculation of max and min distances between keypoints
//        for( int i = 0; i < currentImageDescriptors.rows; i++ )
//        { double dist = matches[i].distance;
//            if( dist < min_dist ) min_dist = dist;
//            if( dist > max_dist ) max_dist = dist;
//        }

//        std::vector< DMatch > good_matches;

//        for( int i = 0; i < currentImageDescriptors.rows; i++ )
//        { if( matches[i].distance < 3*min_dist )
//            { good_matches.push_back( matches[i]); }
//        }

//        Mat img_matches;
//        drawMatches( currentImage, currentImageKeypoints, previousImage, previousImageKeypoints,
//                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//        imwrite(matchingfilename+ to_string(i-1)+"_"+to_string(i)+".png", detectedFeature, compression_params);

//    }

}
