//
//  main.cpp
//  StereoCalibration
//
//  Created by Michael Schwendeman on 8/11/15.
//  Copyright (c) 2015 MSS. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

using namespace cv;
using namespace std;
using namespace boost::filesystem;


struct sort_function
{
    bool operator ()(const path & a,const path & b)
    {
        string aStr = a.filename().string();
        string bStr = b.filename().string();
        //size_t numStartA = aStr.find_last_of("-");
        //size_t numStartB = bStr.find_last_of("-");
        size_t numStartA = aStr.find_last_of("_");
        size_t numStartB = bStr.find_last_of("_");
        long frameNumA = stol(aStr.substr(numStartA+1),nullptr);
        long frameNumB = stol(bStr.substr(numStartB+1),nullptr);
        return frameNumA < frameNumB;
    }
};


int main() {
    // Set paths
    path p1 = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1804UTC_flea34";
    path p2 = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1740UTC_flea35";
    //path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/23Dec2014/1745UTC_PortCalibration";
    //int delay = 11;
    path p = "/Volumes/Data/PAPA/TGTcruise2015/MikeScratchTGT/TGT_StereoVideo/CalibrationDataAndResults/TGTCalibration/23Dec2014_Port_1745UTC";
    int delay = 0;
    //path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/28Dec2014/1605UTC";
    // Set plot options
    bool plotCheckerboard = false;
    bool plotRectified = true;
    if (plotCheckerboard)
    {
        namedWindow("Left Image", CV_WINDOW_AUTOSIZE); //create window for left image
        namedWindow("Right Image", CV_WINDOW_AUTOSIZE); //create window for left image
    }
    if (plotRectified)
    {
        namedWindow("Left Image Rectified", CV_WINDOW_AUTOSIZE); //create window for left image
        namedWindow("Right Image Rectified", CV_WINDOW_AUTOSIZE); //create window for left image
    }
    // Load individual calibrations
    string inputFile1 = p1.string() + "/OpenCVCalibrationResults.yml";
    FileStorage fs1(inputFile1, FileStorage::READ);
    Mat intrinsic1;
    fs1["Intrinsic Matrix"] >> intrinsic1;
    Mat distCoeffs1;
    fs1["Distortion Coefficients"] >> distCoeffs1;
    fs1.release();
    string inputFile2 = p2.string() + "/OpenCVCalibrationResults.yml";
    FileStorage fs2(inputFile2, FileStorage::READ);
    Mat intrinsic2;
    fs2["Intrinsic Matrix"] >> intrinsic2;
    Mat distCoeffs2;
    fs2["Distortion Coefficients"] >> distCoeffs2;
    fs2.release();
    
    // Load and sort stereo image files
    vector<path> allFiles, imageVec1, imageVec2;
    string extStr, fileStr;
    copy(directory_iterator(p),directory_iterator(),back_inserter(allFiles));
    for (vector<path>::const_iterator it (allFiles.begin()); it != allFiles.end(); ++it) {
        extStr = (*it).extension().string();
        fileStr = (*it).filename().string().substr(0,4);
//        fileStr = (*it).filename().string().substr(0,6);
        if (extStr == ".pgm" && fileStr == "left")
            imageVec1.push_back(*it);
        else if (extStr == ".pgm" && fileStr == "righ")
            imageVec2.push_back(*it);
    }
    sort(imageVec1.begin(),imageVec1.end(),sort_function());
    sort(imageVec2.begin(),imageVec2.end(),sort_function());
    
    // Setup checkerboard
    int numCornersHor = 9;
    int numCornersVer = 4;
    int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points1, image_points2;
    vector<Point2f> corners1, corners2;
    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f((j/numCornersHor)*8*25.4, (j%numCornersHor)*8*25.4, 0.0f));
    
    // Detect checkerboard
    Mat image1, image2;
    string filename1, filename2;
    int skip = 1;
    int iframe = 0;
    vector<path>::const_iterator it1 = imageVec1.begin();
    vector<path>::const_iterator it2 = imageVec2.begin();
    
    delay >= 0 ? advance(it2, delay) : advance(it1,-delay);
    while ( it1 != imageVec1.end() && it2 != imageVec2.end())
    {
        if (iframe % skip == 0)
        {
            filename1 = it1->string();
            image1 = imread(filename1,CV_LOAD_IMAGE_GRAYSCALE);
            filename2 = it2->string();
            image2 = imread(filename2,CV_LOAD_IMAGE_GRAYSCALE);
            
            bool found1 = findChessboardCorners(image1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            bool found2 = findChessboardCorners(image2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            
            if(found1 && found2)
            {
                cornerSubPix(image1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                cornerSubPix(image2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                if (plotCheckerboard)
                {
                    drawChessboardCorners(image1, board_sz, corners1, found1);
                    drawChessboardCorners(image2, board_sz, corners2, found2);
                    imshow("Left Image",image1);
                    imshow("Right Image",image2);
                    waitKey(0);
                }
                image_points1.push_back(corners1);
                image_points2.push_back(corners2);
                object_points.push_back(obj);
            }
        }
        it1++;
        it2++;
        iframe++;
    }
    
    // Calculate calibration
    Mat E = Mat(3, 3, CV_32FC1);
    Mat F = Mat(3, 3, CV_32FC1);
    Mat R;
    Mat T;
    
    TermCriteria criteria= TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10000, DBL_EPSILON);
    Size imageSize = image1.size();
    double reprojectError = stereoCalibrate(object_points, image_points1, image_points2, intrinsic1, distCoeffs1, intrinsic2, distCoeffs2, imageSize, R, T, E, F, CV_CALIB_FIX_INTRINSIC, criteria);
    
    
    // RECTIFY
    Mat R1, R2, P1, P2, Q;
    Rect roiRight, roiLeft;
//    stereoRectify(intrinsic1, distCoeffs1,intrinsic2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, imageSize, &roiLeft, &roiRight);
    stereoRectify(intrinsic1, distCoeffs1,intrinsic2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q, 0, 1, imageSize, &roiLeft, &roiRight);
    
    Mat rmap[2][2];
    initUndistortRectifyMap(intrinsic1, distCoeffs1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(intrinsic2, distCoeffs2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    Mat rimg1, rimg2;
    
    it1 = imageVec1.begin();
    it2 = imageVec2.begin();
    
    delay >= 0 ? advance(it2, delay) : advance(it1,-delay);
    while ( it1 != imageVec1.end() && it2 != imageVec2.end())
    {
        if (iframe % skip == 0)
        {
            filename1 = it1->string();
            image1 = imread(filename1,CV_LOAD_IMAGE_GRAYSCALE);
            filename2 = it2->string();
            image2 = imread(filename2,CV_LOAD_IMAGE_GRAYSCALE);
            
            remap(image1, rimg1, rmap[0][0], rmap[0][1], INTER_LINEAR);
            remap(image2, rimg2, rmap[1][0], rmap[1][1], INTER_LINEAR);
            imshow("Left Image Rectified",rimg1);
            imshow("Right Image Rectified",rimg2);
            waitKey(0);
            
        }
        it1++;
        it2++;
        iframe++;
    }
    destroyAllWindows();
    // Save intrinsic matrix and distortion coefficients
    string outputFile;
    outputFile = p.string() + "/OpenCVCalibrationResults.yml";
    FileStorage fs(outputFile, FileStorage::WRITE);
    fs << "Left Intrinsic Matrix" << intrinsic1;
    fs << "Left Distortion Coefficients" << distCoeffs1;
    fs << "Right Intrinsic Matrix" << intrinsic2;
    fs << "Right Distortion Coefficients" << distCoeffs2;
    fs << "Rotation Matrix" << R;
    fs << "Translation Vector" << T;
    fs.release();
    return 0;}