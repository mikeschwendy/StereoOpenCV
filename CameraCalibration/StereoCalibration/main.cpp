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

using namespace cv;
using namespace std;
using namespace boost::filesystem;

int main() {
    // Set paths
    path p1 = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1740UTC_flea35";
    path p2 = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/15Jan2015/1804UTC_flea34";
    path p = "/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/23Dec2014/1745UTC_PortCalibration";
    vector<path> imageVec;
    copy(directory_iterator(p),directory_iterator(),back_inserter(imageVec));
    
    // Load individual calibrations
    
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
        obj.push_back(Point3f((j/numCornersHor)*8*25.4, (j%numCornersHor)*8*25.4, 0.0f));
    

    // Calculate calibration
    Mat E = Mat(3, 3, CV_32FC1);
    Mat F = Mat(3, 3, CV_32FC1);
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    stereoCalibrate(object_points, image_points1, image_points2, intrinsic1, distCoeffs1, intrinsic2, distCoeffs2, image.size(), rvecs, tvecs, E, F, ,CV_CALIB_FIX_INTRINSIC);
    
    return 0;
}