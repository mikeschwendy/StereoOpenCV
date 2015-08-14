//
//  main.cpp
//  CameraCalibration
//
//  Created by Michael Schwendeman on 8/11/15.
//  Copyright (c) 2015 MSS. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>

using namespace cv;
using namespace std;
using namespace boost::filesystem;

int main() {
    // Set plot options
    bool plotUndistort = true;
    bool plotCheckerboard = false;
    if (plotCheckerboard || plotUndistort)
        namedWindow("Image", CV_WINDOW_AUTOSIZE); //create window for left image
    
    // Set path
    path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1740UTC_flea35";
    //path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1804UTC_flea34";
    //path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1750UTC_flea65";
    //path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1834UTC_flea18";
    //path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1811UTC_flea21";
    //path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1825UTC_flea83";
    
    vector<path> imageVec;
    copy(directory_iterator(p),directory_iterator(),back_inserter(imageVec));
    
    // Setup checkerboard
    int numCornersHor = 7;
    int numCornersVer = 4;
    int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;
    vector<Point2f> corners;
    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f((j/numCornersHor)*2*25.4, (j%numCornersHor)*2*25.4, 0.0f));
    
    // Detect checkerboard
    Mat image;
    string filename;
    int skip = 100;
    int iframe = 0;
    for ( vector<path>::const_iterator it = imageVec.begin(); it != imageVec.end(); ++it )
    {
        if ( !(it->extension().empty()) && (it->extension().string() == ".pgm") && (iframe % skip == 0) )
        {
            filename = it->string();
            image = imread(filename,CV_LOAD_IMAGE_GRAYSCALE);
            bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            
            if(found)
            {
                cornerSubPix(image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                if (plotCheckerboard)
                {
                    drawChessboardCorners(image, board_sz, corners, found);
                    imshow("Image",image);
                    waitKey(0);
                }
                image_points.push_back(corners);
                object_points.push_back(obj);
            }
        }
        iframe++;
    }
    
    // Calculate calibration
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 1000, DBL_EPSILON));
    
    // Check undistort
    iframe = 0;
    if (plotUndistort)
    {
        namedWindow("Undistorted", CV_WINDOW_AUTOSIZE); //create window for left image
        Mat imageUndistorted;
        for (vector<path>::const_iterator it (imageVec.begin()); it != imageVec.end(); ++it)
        {
            if ( !(it->extension().empty()) && (it->extension().string() == ".pgm") && (iframe % skip == 0) )
            {
                filename = it->string();
                image = imread(filename);
                undistort(image, imageUndistorted, intrinsic, distCoeffs);
                imshow("Image", image);
                imshow("Undistorted", imageUndistorted);
                waitKey(0);
            }
            iframe++;
        }
        destroyWindow("Undistorted"); //create window for left image
    }
    
    // Close windows
    if (plotCheckerboard || plotUndistort)
        destroyWindow("Image"); //create window for left image
    
    // Save intrinsic matrix and distortion coefficients
    string outputFile;
    outputFile = p.string() + "/OpenCVCalibrationResults.yml";
    FileStorage fs(outputFile, FileStorage::WRITE);
    fs << "Intrinsic Matrix" << intrinsic;
    fs << "Distortion Coefficients" << distCoeffs;
    fs.release();
    return 0;
}
